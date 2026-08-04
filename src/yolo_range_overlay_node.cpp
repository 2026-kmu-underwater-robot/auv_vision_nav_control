#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "auv_vision_nav_control/msg/buoy_detection3_d.hpp"

namespace auv_vision_nav_control
{
using CompressedImage = sensor_msgs::msg::CompressedImage;
using Detection3D = msg::BuoyDetection3D;

class YoloRangeOverlayNode : public rclcpp::Node
{
public:
  YoloRangeOverlayNode() : Node("yolo_range_overlay_node")
  {
    const auto rgb_topic = declare_parameter<std::string>(
      "rgb_image_topic", "/camera/camera/color/image_raw/compressed");
    const auto detection_topic = declare_parameter<std::string>("detection_3d_topic", "/vision/buoy_detection_3d");
    output_topic_ = declare_parameter<std::string>("annotated_image_topic", "/vision/yolo/annotated/compressed");
    max_sync_sec_ = declare_parameter<double>("max_image_detection_sync_sec", 0.35);
    jpeg_quality_ = declare_parameter<int>("annotated_jpeg_quality", 80);
    show_window_ = declare_parameter<bool>("show_window", true);
    window_name_ = declare_parameter<std::string>("window_name", "YOLO Buoy Range");
    image_sub_ = create_subscription<CompressedImage>(rgb_topic, rclcpp::SensorDataQoS(),
      std::bind(&YoloRangeOverlayNode::on_image, this, std::placeholders::_1));
    detection_sub_ = create_subscription<Detection3D>(detection_topic, 10,
      std::bind(&YoloRangeOverlayNode::on_detection, this, std::placeholders::_1));
    output_pub_ = create_publisher<CompressedImage>(output_topic_, rclcpp::QoS(10).reliable());
    RCLCPP_INFO(get_logger(), "RGB input: %s", rgb_topic.c_str());
    RCLCPP_INFO(get_logger(), "3D detection input: %s", detection_topic.c_str());
    RCLCPP_INFO(get_logger(), "Final annotated output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "OpenCV output window: %s", show_window_ ? window_name_.c_str() : "disabled");
  }

private:
  static double seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
  }
  static uint64_t key(const builtin_interfaces::msg::Time & stamp)
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(stamp.sec)) << 32) | stamp.nanosec;
  }

  void on_image(const CompressedImage::SharedPtr image)
  {
    std::scoped_lock lock(mutex_);
    images_.push_back(image);
    while (images_.size() > 60) images_.pop_front();
  }

  void on_detection(const Detection3D::SharedPtr detection)
  {
    std::vector<Detection3D> frame_detections;
    CompressedImage::SharedPtr image;
    {
      std::scoped_lock lock(mutex_);
      auto & frame = pending_[key(detection->header.stamp)];
      const auto duplicate = std::find_if(frame.begin(), frame.end(), [detection](const auto & existing) {
        return existing.detection_index == detection->detection_index;
      });
      if (duplicate == frame.end()) frame.push_back(*detection);
      const uint32_t expected = detection->detections_in_frame;
      if ((expected == 0 && !detection->detected) || (expected > 0 && frame.size() >= expected)) {
        const auto wanted = seconds(detection->header.stamp);
        const auto best = std::min_element(images_.begin(), images_.end(), [wanted](const auto & a, const auto & b) {
          return std::abs(seconds(a->header.stamp) - wanted) < std::abs(seconds(b->header.stamp) - wanted);
        });
        if (best != images_.end() && std::abs(seconds((*best)->header.stamp) - wanted) <= max_sync_sec_) {
          image = *best;
          frame_detections = frame;
        }
        pending_.erase(key(detection->header.stamp));
      }
      while (pending_.size() > 60) pending_.erase(pending_.begin());
    }
    if (image) publish_overlay(*image, frame_detections);
  }

  void publish_overlay(const CompressedImage & source, const std::vector<Detection3D> & detections)
  {
    const auto encoded = cv::Mat(1, static_cast<int>(source.data.size()), CV_8UC1,
      const_cast<uint8_t *>(source.data.data()));
    cv::Mat display = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (display.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unable to decode RGB image for overlay");
      return;
    }
    bool any = false;
    for (const auto & det : detections) {
      if (!det.detected) continue;
      any = true;
      const int x1 = std::lround(det.center_x - det.width / 2.0F);
      const int y1 = std::lround(det.center_y - det.height / 2.0F);
      const int x2 = std::lround(det.center_x + det.width / 2.0F);
      const int y2 = std::lround(det.center_y + det.height / 2.0F);
      const cv::Scalar color = det.range_valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      cv::rectangle(display, {x1, y1}, {x2, y2}, color, 2, cv::LINE_AA);
      std::string label = "class " + std::to_string(det.class_id) + " " +
        cv::format("%.0f%%", det.confidence * 100.0F);
      label += det.range_valid ? "  " + cv::format("%.2f m", det.range_m) : "  distance unavailable";
      cv::putText(display, label, {x1, std::max(22, y1 - 8)}, cv::FONT_HERSHEY_SIMPLEX,
        0.58, color, 2, cv::LINE_AA);
    }
    if (!any) {
      cv::putText(display, "NO DETECTION", {12, 30}, cv::FONT_HERSHEY_SIMPLEX,
        0.8, {0, 0, 255}, 2, cv::LINE_AA);
    }
    if (show_window_) {
      cv::imshow(window_name_, display);
      cv::waitKey(1);
    }
    std::vector<uint8_t> output_data;
    if (!cv::imencode(".jpg", display, output_data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) return;
    CompressedImage output;
    output.header = source.header; output.format = "jpeg"; output.data = std::move(output_data);
    output_pub_->publish(output);
  }

  rclcpp::Subscription<CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<Detection3D>::SharedPtr detection_sub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr output_pub_;
  std::mutex mutex_; std::deque<CompressedImage::SharedPtr> images_;
  std::map<uint64_t, std::vector<Detection3D>> pending_;
  std::string output_topic_, window_name_; double max_sync_sec_; int jpeg_quality_; bool show_window_;
};
}  // namespace auv_vision_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_vision_nav_control::YoloRangeOverlayNode>());
  rclcpp::shutdown();
  return 0;
}
