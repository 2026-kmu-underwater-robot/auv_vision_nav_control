#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "kmu26_auv_planning_vision_control/msg/buoy_detection2_d.hpp"
#include "kmu26_auv_planning_vision_control/msg/buoy_detection3_d.hpp"

namespace kmu26_auv_planning_vision_control
{
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Detection2D = msg::BuoyDetection2D;
using Detection3D = msg::BuoyDetection3D;

class DepthRangeNode : public rclcpp::Node
{
public:
  DepthRangeNode()
  : Node("depth_range_node")
  {
    depth_topic_ = declare_parameter<std::string>(
      "depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    const auto depth_info_topic = declare_parameter<std::string>(
      "depth_camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info");
    const auto detection_topic = declare_parameter<std::string>(
      "detection_topic", "/vision/buoy_detection_2d");
    output_topic_ = declare_parameter<std::string>(
      "detection_3d_topic", "/vision/buoy_detection_3d");
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    max_sync_sec_ = declare_parameter<double>("max_detection_depth_sync_sec", 0.25);
    roi_scale_ = declare_parameter<double>("roi_scale", 0.55);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.20);
    max_range_m_ = declare_parameter<double>("max_range_m", 6.0);
    min_valid_pixels_ = declare_parameter<int>("min_valid_depth_pixels", 8);
    max_robust_sigma_m_ = declare_parameter<double>("max_robust_sigma_m", 0.20);
    min_outlier_gate_m_ = declare_parameter<double>("min_outlier_gate_m", 0.08);
    sample_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("sample_stride", 1)));

    depth_sub_ = create_subscription<Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthRangeNode::on_depth, this, std::placeholders::_1));
    depth_info_sub_ = create_subscription<CameraInfo>(
      depth_info_topic, rclcpp::SensorDataQoS(),
      [this](CameraInfo::SharedPtr msg) {
        std::scoped_lock lock(data_mutex_); depth_info_ = *msg;
      });
    detection_sub_ = create_subscription<Detection2D>(
      detection_topic, 10, std::bind(&DepthRangeNode::on_detection, this, std::placeholders::_1));
    output_pub_ = create_publisher<Detection3D>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "Aligned RealSense depth input: %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "YOLO 2D input: %s", detection_topic.c_str());
    RCLCPP_INFO(get_logger(), "3D detection output: %s", output_topic_.c_str());
  }

private:
  struct PointSample
  {
    double x;
    double y;
    double z;
    float range;
  };

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
  }

  static float median(std::vector<float> values)
  {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  static double median(std::vector<double> values)
  {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  void on_depth(const Image::SharedPtr image)
  {
    std::scoped_lock lock(data_mutex_);
    depth_frames_.push_back(image);
    while (depth_frames_.size() > 120) depth_frames_.pop_front();
  }

  Detection3D make_output(const Detection2D & input) const
  {
    Detection3D output;
    output.header = input.header;
    output.detected = input.detected;
    output.range_valid = false;
    output.class_id = input.class_id;
    output.confidence = input.confidence;
    output.center_x = input.center_x;
    output.center_y = input.center_y;
    output.width = input.width;
    output.height = input.height;
    output.image_width = input.image_width;
    output.image_height = input.image_height;
    output.detections_in_frame = input.detections_in_frame;
    output.detection_index = input.detection_index;
    return output;
  }

  void publish_invalid(const Detection2D & input, const char * reason)
  {
    output_pub_->publish(make_output(input));
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "Depth unavailable: %s", reason);
  }

  void on_detection(const Detection2D::SharedPtr detection)
  {
    if (!detection->detected) {
      publish_invalid(*detection, "no YOLO detection");
      return;
    }

    Image::SharedPtr depth;
    CameraInfo depth_info;
    {
      std::scoped_lock lock(data_mutex_);
      if (depth_frames_.empty() || !depth_info_) {
        publish_invalid(*detection, "depth frame or CameraInfo missing");
        return;
      }
      const double wanted = stamp_seconds(detection->header.stamp);
      const auto best = std::min_element(
        depth_frames_.begin(), depth_frames_.end(), [wanted](const auto & a, const auto & b) {
          return std::abs(stamp_seconds(a->header.stamp) - wanted) <
                 std::abs(stamp_seconds(b->header.stamp) - wanted);
        });
      if (std::abs(stamp_seconds((*best)->header.stamp) - wanted) > max_sync_sec_) {
        publish_invalid(*detection, "RGB/depth timestamp mismatch");
        return;
      }
      depth = *best;
      depth_info = *depth_info_;
    }
    calculate_range(*detection, depth, depth_info);
  }

  double depth_at(const cv::Mat & image, int row, int col, const std::string & encoding) const
  {
    if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 || encoding == "mono16") {
      return static_cast<double>(image.at<uint16_t>(row, col)) * depth_scale_;
    }
    if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      return static_cast<double>(image.at<float>(row, col));
    }
    return 0.0;
  }

  void calculate_range(
    const Detection2D & detection, const Image::SharedPtr & depth_msg,
    const CameraInfo & depth_info)
  {
    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      depth_msg->encoding != "mono16" &&
      depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
      publish_invalid(detection, "unsupported depth encoding");
      return;
    }

    const double fx = depth_info.k[0];
    const double fy = depth_info.k[4];
    const double cx = depth_info.k[2];
    const double cy = depth_info.k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      publish_invalid(detection, "invalid camera intrinsics");
      return;
    }
    const cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
    if (detection.image_width == 0 || detection.image_height == 0) {
      publish_invalid(detection, "YOLO image dimensions missing");
      return;
    }

    // Aligned depth has the color camera's pixel grid, so bbox coordinates map directly.
    // Scaling retains correctness when the detector received a resized color stream.
    const double scale_x = static_cast<double>(depth.cols) / detection.image_width;
    const double scale_y = static_cast<double>(depth.rows) / detection.image_height;
    const double half_width = detection.width * roi_scale_ * scale_x / 2.0;
    const double half_height = detection.height * roi_scale_ * scale_y / 2.0;
    const int u0 = std::max(0, static_cast<int>(std::floor(detection.center_x * scale_x - half_width)));
    const int u1 = std::min(depth.cols, static_cast<int>(std::ceil(detection.center_x * scale_x + half_width)));
    const int v0 = std::max(0, static_cast<int>(std::floor(detection.center_y * scale_y - half_height)));
    const int v1 = std::min(depth.rows, static_cast<int>(std::ceil(detection.center_y * scale_y + half_height)));
    if (u0 >= u1 || v0 >= v1) {
      publish_invalid(detection, "YOLO ROI outside aligned depth image");
      return;
    }
    std::vector<PointSample> samples;
    samples.reserve(static_cast<size_t>((u1 - u0) * (v1 - v0) / (sample_stride_ * sample_stride_)));

    for (int v = v0; v < v1; v += sample_stride_) {
      for (int u = u0; u < u1; u += sample_stride_) {
        const double z = depth_at(depth, v, u, depth_msg->encoding);
        if (!std::isfinite(z) || z < min_range_m_ || z > max_range_m_) continue;
        const double x = (static_cast<double>(u) - cx) * z / fx;
        const double y = (static_cast<double>(v) - cy) * z / fy;
        samples.push_back({x, y, z, static_cast<float>(std::sqrt(x * x + y * y + z * z))});
      }
    }

    if (static_cast<int>(samples.size()) < min_valid_pixels_) {
      publish_invalid(detection, "too few depth pixels in YOLO ROI");
      return;
    }

    std::vector<float> ranges;
    ranges.reserve(samples.size());
    for (const auto & sample : samples) ranges.push_back(sample.range);
    const float center_range = median(ranges);
    std::vector<float> deviations;
    deviations.reserve(ranges.size());
    for (const float range : ranges) deviations.push_back(std::abs(range - center_range));
    const float mad = median(deviations);
    const float robust_sigma = 1.4826F * mad;
    const float gate = std::max(static_cast<float>(min_outlier_gate_m_), 3.0F * robust_sigma);

    std::vector<double> xs, ys, zs;
    std::vector<float> filtered_ranges;
    for (const auto & sample : samples) {
      if (std::abs(sample.range - center_range) > gate) continue;
      xs.push_back(sample.x); ys.push_back(sample.y); zs.push_back(sample.z);
      filtered_ranges.push_back(sample.range);
    }
    if (static_cast<int>(filtered_ranges.size()) < min_valid_pixels_ || robust_sigma > max_robust_sigma_m_) {
      publish_invalid(detection, "depth cluster failed robust quality gate");
      return;
    }

    Detection3D output = make_output(detection);
    output.range_valid = true;
    output.point_camera.x = median(xs);
    output.point_camera.y = median(ys);
    output.point_camera.z = median(zs);
    output.range_m = median(filtered_ranges);
    output.depth_stddev_m = robust_sigma;
    output.valid_depth_pixels = static_cast<uint32_t>(filtered_ranges.size());
    output_pub_->publish(output);
  }

  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<Detection2D>::SharedPtr detection_sub_;
  rclcpp::Publisher<Detection3D>::SharedPtr output_pub_;
  std::mutex data_mutex_;
  std::deque<Image::SharedPtr> depth_frames_;
  std::optional<CameraInfo> depth_info_;
  std::string depth_topic_;
  std::string output_topic_;
  double depth_scale_;
  double max_sync_sec_;
  double roi_scale_;
  double min_range_m_;
  double max_range_m_;
  double max_robust_sigma_m_;
  double min_outlier_gate_m_;
  int min_valid_pixels_;
  int sample_stride_;
};
}  // namespace kmu26_auv_planning_vision_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kmu26_auv_planning_vision_control::DepthRangeNode>());
  rclcpp::shutdown();
  return 0;
}
