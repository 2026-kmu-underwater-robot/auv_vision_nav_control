#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kmu26_auv_planning_vision_control/msg/buoy_detection3_d.hpp"
#include "kmu26_auv_planning_vision_control/msg/buoy_track.hpp"
#include "kmu26_auv_planning_vision_control/msg/buoy_track_array.hpp"

namespace kmu26_auv_planning_vision_control
{

using Detection3D = msg::BuoyDetection3D;
using TrackMessage = msg::BuoyTrack;
using TrackArray = msg::BuoyTrackArray;
using Point = geometry_msgs::msg::Point;

namespace
{
double point_distance(const Point & a, const Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

double median(std::vector<double> values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 != 0) {
    return *middle;
  }
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + upper);
}

tf2::Quaternion normalized_quaternion(const geometry_msgs::msg::Quaternion & input)
{
  tf2::Quaternion output(input.x, input.y, input.z, input.w);
  if (output.length2() < 1e-12) {
    throw std::runtime_error("zero quaternion");
  }
  output.normalize();
  return output;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto quaternion = normalized_quaternion(orientation);
  const double x = quaternion.x();
  const double y = quaternion.y();
  const double z = quaternion.z();
  const double w = quaternion.w();
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class BuoyCoordinateMapper : public rclcpp::Node
{
public:
  BuoyCoordinateMapper()
  : Node("buoy_coordinate_mapper"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    arena_start_frame_topic_ = declare_parameter<std::string>(
      "arena_start_frame_topic", "/guided/start_frame");
    detection_topic_ = declare_parameter<std::string>(
      "detection_3d_topic", "/vision/buoy_detection_3d");
    tracks_topic_ = declare_parameter<std::string>(
      "tracks_topic", "/mission/buoy_tracks");
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/mission/buoy_observation");
    vision_search_topic_ = declare_parameter<std::string>(
      "vision_search_topic", "/homing/vision_search_active");
    target_confirmed_topic_ = declare_parameter<std::string>(
      "target_confirmed_topic", "/vision/target_confirmed");

    arena_length_m_ = declare_parameter<double>("arena_length_m", 10.0);
    arena_width_m_ = declare_parameter<double>("arena_width_m", 15.0);
    arena_depth_m_ = declare_parameter<double>("arena_depth_m", 11.0);
    arena_start_corner_ = declare_parameter<std::string>(
      "arena_start_corner", "bottom_left");

    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    association_distance_m_ = declare_parameter<double>("association_distance_m", 1.0);
    observation_outlier_distance_m_ = declare_parameter<double>(
      "observation_outlier_distance_m", 0.6);
    track_buffer_size_ = declare_parameter<int>("track_buffer_size", 40);
    min_confirm_observations_ = declare_parameter<int>("min_confirm_observations", 6);
    max_position_rms_m_ = declare_parameter<double>("max_position_rms_m", 0.30);
    depth_variance_floor_m2_ = declare_parameter<double>(
      "depth_variance_floor_m2", 0.0025);
    odom_buffer_sec_ = declare_parameter<double>("odom_buffer_sec", 3.0);
    max_detection_odom_gap_sec_ = declare_parameter<double>(
      "max_detection_odom_gap_sec", 0.10);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.08);
    handoff_confirm_hits_ = declare_parameter<int>("handoff_confirm_hits", 4);
    handoff_detection_timeout_sec_ = declare_parameter<double>(
      "handoff_detection_timeout_sec", 0.7);
    handoff_detection_consistency_m_ = declare_parameter<double>(
      "handoff_detection_consistency_m", 0.75);
    validate_parameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuoyCoordinateMapper::on_odom, this, std::placeholders::_1));
    arena_start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      arena_start_frame_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyCoordinateMapper::on_arena_start_frame, this, std::placeholders::_1));
    detection_sub_ = create_subscription<Detection3D>(
      detection_topic_, 20,
      std::bind(&BuoyCoordinateMapper::on_detection, this, std::placeholders::_1));
    vision_search_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_search_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyCoordinateMapper::on_vision_search, this, std::placeholders::_1));

    tracks_pub_ = create_publisher<TrackArray>(
      tracks_topic_, rclcpp::QoS(1).reliable().transient_local());
    observation_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      observation_topic_, 20);
    target_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
      target_confirmed_topic_, rclcpp::QoS(1).reliable().transient_local());

    publish_target_confirmed(false);
    RCLCPP_INFO(
      get_logger(),
      "Buoy coordinate mapper ready: detection=%s odom=%s tracks=%s, "
      "arena-local x=[0.00, %.2f] width=%.2f z=[%.2f, 0.00]; waiting for %s",
      detection_topic_.c_str(), odom_topic_.c_str(), tracks_topic_.c_str(),
      arena_length_m_, arena_width_m_, -arena_depth_m_,
      arena_start_frame_topic_.c_str());
  }

private:
  struct TimedPose
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Point position;
    tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};
  };

  struct Observation
  {
    Point point;
    double weight{1.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct Track
  {
    uint32_t id{0};
    int class_id{-1};
    std::deque<Observation> observations;
    Point position;
    double rms{std::numeric_limits<double>::infinity()};
    size_t inliers{0};
    bool confirmed{false};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  };

  void validate_parameters() const
  {
    if (
      arena_length_m_ <= 0.0 || arena_width_m_ <= 0.0 || arena_depth_m_ <= 0.0 ||
      arena_start_frame_topic_.empty() ||
      association_distance_m_ <= 0.0 || observation_outlier_distance_m_ <= 0.0 ||
      track_buffer_size_ < 1 || min_confirm_observations_ < 1 ||
      min_confirm_observations_ > track_buffer_size_ ||
      max_position_rms_m_ < 0.0 || depth_variance_floor_m2_ <= 0.0 ||
      odom_buffer_sec_ <= 0.0 || max_detection_odom_gap_sec_ <= 0.0 ||
      tf_lookup_timeout_sec_ < 0.0 || handoff_confirm_hits_ < 1 ||
      handoff_detection_timeout_sec_ <= 0.0 ||
      handoff_detection_consistency_m_ <= 0.0)
    {
      throw std::invalid_argument("buoy coordinate mapper parameters are invalid");
    }
    if (
      arena_start_corner_ != "bottom_left" &&
      arena_start_corner_ != "bottom_right")
    {
      throw std::invalid_argument(
              "arena_start_corner must be bottom_left or bottom_right");
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id.empty() || message->child_frame_id.empty()) {
      return;
    }
    if (
      arena_frame_ready_ &&
      message->header.frame_id != arena_parent_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored odometry in '%s'; arena start frame uses '%s'",
        message->header.frame_id.c_str(), arena_parent_frame_.c_str());
      return;
    }
    const rclcpp::Time stamp(message->header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() <= 0) {
      return;
    }
    if (!odom_buffer_.empty() && stamp <= rclcpp::Time(odom_buffer_.back().header.stamp)) {
      odom_buffer_.clear();
    }
    odom_frame_ = message->header.frame_id;
    body_frame_ = message->child_frame_id;
    odom_buffer_.push_back(*message);
    while (
      odom_buffer_.size() > 2 &&
      (stamp - rclcpp::Time(odom_buffer_.front().header.stamp)).seconds() >
      odom_buffer_sec_)
    {
      odom_buffer_.pop_front();
    }
  }

  void on_arena_start_frame(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Ignored /guided/start_frame with an empty parent frame");
      return;
    }
    if (!odom_frame_.empty() && message->header.frame_id != odom_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignored /guided/start_frame in '%s'; odometry uses '%s'",
        message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    const auto & origin = message->pose.position;
    try {
      if (
        !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
        !std::isfinite(origin.z))
      {
        throw std::runtime_error("non-finite origin");
      }
      arena_origin_ = origin;
      arena_yaw_rad_ = yaw_from_quaternion(message->pose.orientation);
      arena_parent_frame_ = message->header.frame_id;
      arena_frame_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Arena frame received: origin=(%.3f, %.3f, %.3f), yaw=%.2f deg",
        arena_origin_.x, arena_origin_.y, arena_origin_.z,
        arena_yaw_rad_ * 180.0 / 3.14159265358979323846);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Ignored invalid /guided/start_frame: %s", error.what());
    }
  }

  void on_vision_search(const std_msgs::msg::Bool::SharedPtr message)
  {
    vision_search_active_ = message->data;
    handoff_detection_hits_ = 0;
    last_handoff_detection_stamp_.reset();
    last_handoff_camera_point_.reset();
    target_confirmation_sent_ = false;
    publish_target_confirmed(false);
  }

  void on_detection(const Detection3D::SharedPtr detection)
  {
    update_handoff_confirmation(*detection);
    if (
      !detection->detected || !detection->range_valid ||
      detection->class_id != buoy_class_id_ || detection->header.frame_id.empty())
    {
      return;
    }
    const auto odom_point = detection_to_odom(*detection);
    if (!odom_point) {
      return;
    }
    if (!arena_frame_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s before storing odom buoy tracks",
        arena_start_frame_topic_.c_str());
      return;
    }
    if (!inside_arena(*odom_point)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected buoy point outside arena: odom=(%.2f %.2f %.2f)",
        odom_point->x, odom_point->y, odom_point->z);
      return;
    }
    const double variance = std::max(
      depth_variance_floor_m2_,
      static_cast<double>(detection->depth_stddev_m) * detection->depth_stddev_m);
    const double weight =
      std::max(0.05, static_cast<double>(detection->confidence)) / variance;
    associate_track(
      *odom_point, detection->class_id,
      rclcpp::Time(detection->header.stamp, RCL_ROS_TIME), weight);
    publish_observation(*odom_point, detection->header.stamp);
    publish_tracks();
  }

  std::optional<TimedPose> interpolated_odom(const rclcpp::Time & wanted) const
  {
    if (odom_buffer_.empty()) {
      return std::nullopt;
    }
    const auto first_stamp = rclcpp::Time(odom_buffer_.front().header.stamp);
    const auto last_stamp = rclcpp::Time(odom_buffer_.back().header.stamp);
    if (
      (first_stamp - wanted).seconds() > max_detection_odom_gap_sec_ ||
      (wanted - last_stamp).seconds() > max_detection_odom_gap_sec_)
    {
      return std::nullopt;
    }
    auto upper = std::lower_bound(
      odom_buffer_.begin(), odom_buffer_.end(), wanted,
      [](const nav_msgs::msg::Odometry & odom, const rclcpp::Time & stamp) {
        return rclcpp::Time(odom.header.stamp) < stamp;
      });
    if (upper == odom_buffer_.begin()) {
      return make_timed_pose(*upper);
    }
    if (upper == odom_buffer_.end()) {
      return make_timed_pose(odom_buffer_.back());
    }
    const auto lower = std::prev(upper);
    const rclcpp::Time t0(lower->header.stamp);
    const rclcpp::Time t1(upper->header.stamp);
    const double span = (t1 - t0).seconds();
    if (span <= 0.0 || span > max_detection_odom_gap_sec_) {
      return std::nullopt;
    }
    const double alpha = std::clamp((wanted - t0).seconds() / span, 0.0, 1.0);
    TimedPose output;
    output.stamp = wanted;
    output.position.x =
      lower->pose.pose.position.x +
      alpha * (upper->pose.pose.position.x - lower->pose.pose.position.x);
    output.position.y =
      lower->pose.pose.position.y +
      alpha * (upper->pose.pose.position.y - lower->pose.pose.position.y);
    output.position.z =
      lower->pose.pose.position.z +
      alpha * (upper->pose.pose.position.z - lower->pose.pose.position.z);
    try {
      const auto q0 = normalized_quaternion(lower->pose.pose.orientation);
      const auto q1 = normalized_quaternion(upper->pose.pose.orientation);
      output.orientation = q0.slerp(q1, alpha);
      output.orientation.normalize();
    } catch (const std::exception &) {
      return std::nullopt;
    }
    return output;
  }

  static TimedPose make_timed_pose(const nav_msgs::msg::Odometry & odom)
  {
    TimedPose output;
    output.stamp = rclcpp::Time(odom.header.stamp);
    output.position = odom.pose.pose.position;
    output.orientation = normalized_quaternion(odom.pose.pose.orientation);
    return output;
  }

  std::optional<Point> detection_to_odom(const Detection3D & detection)
  {
    const auto pose = interpolated_odom(
      rclcpp::Time(detection.header.stamp, RCL_ROS_TIME));
    if (!pose || body_frame_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No odometry sample close enough to the buoy image timestamp");
      return std::nullopt;
    }
    try {
      const auto body_from_camera = tf_buffer_.lookupTransform(
        body_frame_, detection.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
      const auto camera_rotation =
        normalized_quaternion(body_from_camera.transform.rotation);
      const tf2::Vector3 camera_point(
        detection.point_camera.x, detection.point_camera.y, detection.point_camera.z);
      const auto & translation = body_from_camera.transform.translation;
      const tf2::Vector3 body_point =
        tf2::quatRotate(camera_rotation, camera_point) +
        tf2::Vector3(translation.x, translation.y, translation.z);
      const tf2::Vector3 odom_offset =
        tf2::quatRotate(pose->orientation, body_point);
      Point output;
      output.x = pose->position.x + odom_offset.x();
      output.y = pose->position.y + odom_offset.y();
      output.z = pose->position.z + odom_offset.z();
      return output;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Camera-to-base TF unavailable: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Buoy coordinate transform rejected: %s", error.what());
    }
    return std::nullopt;
  }

  void associate_track(
    const Point & point, int class_id, const rclcpp::Time & stamp, double weight)
  {
    size_t best_index = tracks_.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const auto & track = tracks_[i];
      if (track.class_id != class_id) {
        continue;
      }
      const double distance = point_distance(point, track.position);
      const double adaptive_gate = std::isfinite(track.rms) ?
        std::max(association_distance_m_, 3.0 * track.rms) :
        association_distance_m_;
      if (distance <= adaptive_gate && distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }
    if (best_index == tracks_.size()) {
      Track track;
      track.id = next_track_id_++;
      track.class_id = class_id;
      track.position = point;
      track.last_seen = stamp;
      tracks_.push_back(track);
      best_index = tracks_.size() - 1;
      RCLCPP_INFO(
        get_logger(),
        "New buoy candidate id=%u odom=(%.2f %.2f %.2f)",
        tracks_[best_index].id, point.x, point.y, point.z);
    }
    auto & track = tracks_[best_index];
    track.last_seen = stamp;
    track.observations.push_back({point, weight, stamp});
    while (static_cast<int>(track.observations.size()) > track_buffer_size_) {
      track.observations.pop_front();
    }
    update_track(track);
  }

  void update_track(Track & track)
  {
    std::vector<double> xs, ys, zs;
    xs.reserve(track.observations.size());
    ys.reserve(track.observations.size());
    zs.reserve(track.observations.size());
    for (const auto & observation : track.observations) {
      xs.push_back(observation.point.x);
      ys.push_back(observation.point.y);
      zs.push_back(observation.point.z);
    }
    Point center;
    center.x = median(xs);
    center.y = median(ys);
    center.z = median(zs);

    Point estimate;
    double weight_sum = 0.0;
    size_t inliers = 0;
    for (const auto & observation : track.observations) {
      if (point_distance(observation.point, center) > observation_outlier_distance_m_) {
        continue;
      }
      estimate.x += observation.weight * observation.point.x;
      estimate.y += observation.weight * observation.point.y;
      estimate.z += observation.weight * observation.point.z;
      weight_sum += observation.weight;
      ++inliers;
    }
    if (inliers == 0 || weight_sum <= 0.0) {
      return;
    }
    estimate.x /= weight_sum;
    estimate.y /= weight_sum;
    estimate.z /= weight_sum;
    double squared_residual_sum = 0.0;
    for (const auto & observation : track.observations) {
      if (point_distance(observation.point, center) <= observation_outlier_distance_m_) {
        const double residual = point_distance(observation.point, estimate);
        squared_residual_sum += residual * residual;
      }
    }
    track.position = estimate;
    track.inliers = inliers;
    track.rms = std::sqrt(squared_residual_sum / static_cast<double>(inliers));
    track.confirmed =
      track.confirmed ||
      (inliers >= static_cast<size_t>(min_confirm_observations_) &&
      track.rms <= max_position_rms_m_);
  }

  void update_handoff_confirmation(const Detection3D & detection)
  {
    if (!vision_search_active_ || target_confirmation_sent_) {
      return;
    }
    const Point camera_point = detection.point_camera;
    const bool valid =
      detection.detected && detection.range_valid &&
      detection.class_id == buoy_class_id_ &&
      std::isfinite(camera_point.x) && std::isfinite(camera_point.y) &&
      std::isfinite(camera_point.z) && detection.range_m > 0.0F;
    if (!valid) {
      handoff_detection_hits_ = 0;
      last_handoff_detection_stamp_.reset();
      last_handoff_camera_point_.reset();
      return;
    }
    const rclcpp::Time stamp(detection.header.stamp, RCL_ROS_TIME);
    if (last_handoff_detection_stamp_ && stamp == *last_handoff_detection_stamp_) {
      return;
    }
    const bool timed_out =
      last_handoff_detection_stamp_ &&
      (stamp - *last_handoff_detection_stamp_).seconds() >
      handoff_detection_timeout_sec_;
    const bool jumped =
      last_handoff_camera_point_ &&
      point_distance(camera_point, *last_handoff_camera_point_) >
      handoff_detection_consistency_m_;
    handoff_detection_hits_ =
      (timed_out || jumped) ? 1 : handoff_detection_hits_ + 1;
    last_handoff_detection_stamp_ = stamp;
    last_handoff_camera_point_ = camera_point;
    if (handoff_detection_hits_ >= handoff_confirm_hits_) {
      target_confirmation_sent_ = true;
      publish_target_confirmed(true);
      RCLCPP_INFO(
        get_logger(),
        "Vision target confirmed after %d consistent YOLO+Depth frames",
        handoff_detection_hits_);
    }
  }

  void publish_tracks()
  {
    if (odom_frame_.empty()) {
      return;
    }
    TrackArray output;
    output.header.stamp = now();
    output.header.frame_id = odom_frame_;
    for (const auto & track : tracks_) {
      TrackMessage item;
      item.id = track.id;
      item.class_id = track.class_id;
      // Legacy field name: the array header is authoritative; this point is in odom.
      item.position_mission = track.position;
      item.position_rms_m = static_cast<float>(track.rms);
      item.observation_count = static_cast<uint32_t>(track.inliers);
      item.status = track.confirmed ?
        TrackMessage::CONFIRMED : TrackMessage::CANDIDATE;
      item.last_seen = static_cast<builtin_interfaces::msg::Time>(track.last_seen);
      output.tracks.push_back(item);
    }
    tracks_pub_->publish(output);
  }

  void publish_observation(
    const Point & point, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PointStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = odom_frame_;
    output.point = point;
    observation_pub_->publish(output);
  }

  void publish_target_confirmed(bool confirmed)
  {
    std_msgs::msg::Bool output;
    output.data = confirmed;
    target_confirmed_pub_->publish(output);
  }

  Point odom_to_arena(const Point & odom) const
  {
    const double cosine = std::cos(arena_yaw_rad_);
    const double sine = std::sin(arena_yaw_rad_);
    const double dx = odom.x - arena_origin_.x;
    const double dy = odom.y - arena_origin_.y;
    Point output;
    output.x = cosine * dx + sine * dy;
    output.y = -sine * dx + cosine * dy;
    output.z = odom.z - arena_origin_.z;
    return output;
  }

  bool inside_arena(const Point & point) const
  {
    if (!arena_frame_ready_) {
      return false;
    }
    const Point arena = odom_to_arena(point);
    const double y_min =
      arena_start_corner_ == "bottom_left" ? -arena_width_m_ : 0.0;
    const double y_max =
      arena_start_corner_ == "bottom_left" ? 0.0 : arena_width_m_;
    return
      arena.x >= 0.0 && arena.x <= arena_length_m_ &&
      arena.y >= y_min && arena.y <= y_max &&
      arena.z >= -arena_depth_m_ && arena.z <= 0.0;
  }

  std::string odom_topic_, arena_start_frame_topic_;
  std::string detection_topic_, tracks_topic_, observation_topic_;
  std::string vision_search_topic_, target_confirmed_topic_;
  std::string arena_start_corner_;
  std::string odom_frame_, body_frame_, arena_parent_frame_;
  int buoy_class_id_{0}, track_buffer_size_{40}, min_confirm_observations_{6};
  int handoff_confirm_hits_{4}, handoff_detection_hits_{0};
  double association_distance_m_{1.0}, observation_outlier_distance_m_{0.6};
  double max_position_rms_m_{0.30}, depth_variance_floor_m2_{0.0025};
  double odom_buffer_sec_{3.0}, max_detection_odom_gap_sec_{0.10};
  double tf_lookup_timeout_sec_{0.08}, handoff_detection_timeout_sec_{0.7};
  double handoff_detection_consistency_m_{0.75};
  double arena_length_m_{10.0}, arena_width_m_{15.0}, arena_depth_m_{11.0};
  Point arena_origin_;
  double arena_yaw_rad_{0.0};
  bool arena_frame_ready_{false};
  bool vision_search_active_{false}, target_confirmation_sent_{false};
  uint32_t next_track_id_{1};
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  std::vector<Track> tracks_;
  std::optional<rclcpp::Time> last_handoff_detection_stamp_;
  std::optional<Point> last_handoff_camera_point_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arena_start_frame_sub_;
  rclcpp::Subscription<Detection3D>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_search_sub_;
  rclcpp::Publisher<TrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr observation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_confirmed_pub_;
};

}  // namespace kmu26_auv_planning_vision_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      kmu26_auv_planning_vision_control::BuoyCoordinateMapper>());
  rclcpp::shutdown();
  return 0;
}
