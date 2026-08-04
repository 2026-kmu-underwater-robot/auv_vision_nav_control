#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "auv_vision_nav_control/msg/buoy_detection2_d.hpp"
#include "auv_vision_nav_control/msg/buoy_track.hpp"
#include "auv_vision_nav_control/msg/buoy_track_array.hpp"

namespace auv_vision_nav_control
{

using Detection2D = msg::BuoyDetection2D;
using TrackMessage = msg::BuoyTrack;
using TrackArray = msg::BuoyTrackArray;
using Point = geometry_msgs::msg::Point;

namespace
{
double point_distance(const Point & a, const Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

double horizontal_distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::optional<double> yaw_from_quaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (
    !std::isfinite(norm_squared) || norm_squared < 1.0e-12 ||
    !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return std::nullopt;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = orientation.x * inverse_norm;
  const double y = orientation.y * inverse_norm;
  const double z = orientation.z * inverse_norm;
  const double w = orientation.w * inverse_norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class BuoyMissionManager : public rclcpp::Node
{
public:
  BuoyMissionManager()
  : Node("buoy_mission_manager")
  {
    declare_topics();
    declare_search_parameters();
    declare_tracking_parameters();
    declare_control_parameters();
    validate_parameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuoyMissionManager::on_odom, this, std::placeholders::_1));
    arena_start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      arena_start_frame_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_arena_start_frame, this, std::placeholders::_1));
    detection_2d_sub_ = create_subscription<Detection2D>(
      detection_2d_topic_, 20,
      std::bind(&BuoyMissionManager::on_detection_2d, this, std::placeholders::_1));
    tracks_sub_ = create_subscription<TrackArray>(
      buoy_tracks_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_tracks, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      mission_start_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_mission_start, this, std::placeholders::_1));
    target_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_complete_topic_, 10,
      std::bind(&BuoyMissionManager::on_target_complete, this, std::placeholders::_1));
    target_failed_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_failed_topic_, 10,
      std::bind(&BuoyMissionManager::on_target_failed, this, std::placeholders::_1));
    fcu_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      fcu_state_topic_, 10,
      [this](mavros_msgs::msg::State::SharedPtr message) {fcu_state_ = *message;});
    visual_state_sub_ = create_subscription<std_msgs::msg::String>(
      visual_state_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr message) {visual_controller_state_ = message->data;});
    vision_search_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_search_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_vision_search, this, std::placeholders::_1));
    control_granted_sub_ = create_subscription<std_msgs::msg::Bool>(
      control_granted_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_control_granted, this, std::placeholders::_1));
    target_confirmed_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_confirmed_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyMissionManager::on_target_confirmed, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::String>(
      mission_state_topic_, rclcpp::QoS(1).reliable().transient_local());
    selected_bbox_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      selected_bbox_topic_, 10);
    vision_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      vision_enable_topic_, rclcpp::QoS(1).reliable().transient_local());
    waypoint_debug_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      waypoint_debug_topic_, 10);
    waypoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      waypoint_topic_, 10);
    arena_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&BuoyMissionManager::on_timer, this));

    publish_vision_enable(false);
    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "Mission manager ready: arena-local x=[0.00, %.2f] width=%.2f "
      "z=[%.2f, 0.00], frame=%s, dry_run=%s; waiting for %s and start flag on %s",
      search_length_, search_width_, -search_max_depth_,
      arena_frame_.c_str(),
      dry_run_ ? "true" : "false",
      arena_start_frame_topic_.c_str(),
      mission_start_topic_.c_str());
    if (dry_run_) {
      RCLCPP_WARN(
        get_logger(),
        "dry_run=true: tracking and state transitions run, but no external waypoint is sent");
    }
  }

private:
  enum class MissionState
  {
    IDLE,
    WAIT_FOR_TARGET,
    GUIDED_APPROACH,
    HANDOFF_WAIT,
    VISUAL_SERVO,
    COMPLETE,
    FAILSAFE
  };

  struct BuoyTrack
  {
    uint32_t id{0};
    int class_id{-1};
    Point position;
    double rms{std::numeric_limits<double>::infinity()};
    size_t inliers{0};
    bool confirmed{false};
    uint8_t status{TrackMessage::CANDIDATE};
    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
    std::optional<std::pair<double, double>> last_image_center;
  };

  struct DetectionFrame
  {
    uint32_t expected{0};
    std::vector<Detection2D> detections;
  };

  void declare_topics()
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    arena_start_frame_topic_ = declare_parameter<std::string>(
      "arena_start_frame_topic", "/guided/start_frame");
    arena_frame_ = declare_parameter<std::string>("arena_frame", "arena");
    detection_2d_topic_ = declare_parameter<std::string>(
      "detection_2d_topic", "/vision/buoy_detection_2d");
    mission_start_topic_ = declare_parameter<std::string>(
      "mission_start_topic", "/mission/start");
    target_complete_topic_ = declare_parameter<std::string>(
      "target_complete_topic", "/mission/target_complete");
    target_failed_topic_ = declare_parameter<std::string>(
      "target_failed_topic", "/mission/target_failed");
    mission_state_topic_ = declare_parameter<std::string>(
      "mission_state_topic", "/mission/state");
    buoy_tracks_topic_ = declare_parameter<std::string>(
      "buoy_tracks_topic", "/mission/buoy_tracks");
    selected_bbox_topic_ = declare_parameter<std::string>(
      "selected_bbox_topic", "/vision/buoy_bbox");
    vision_enable_topic_ = declare_parameter<std::string>(
      "vision_enable_topic", "/mission/control_enable");
    waypoint_debug_topic_ = declare_parameter<std::string>(
      "waypoint_debug_topic", "/mission/active_waypoint");
    waypoint_topic_ = declare_parameter<std::string>(
      "waypoint_topic", "/waypoint");
    fcu_state_topic_ = declare_parameter<std::string>("fcu_state_topic", "/mavros/state");
    set_mode_service_ = declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");
    visual_state_topic_ = declare_parameter<std::string>(
      "visual_state_topic", "/mission/visual_state");
    vision_search_topic_ = declare_parameter<std::string>(
      "vision_search_topic", "/homing/vision_search_active");
    target_confirmed_topic_ = declare_parameter<std::string>(
      "target_confirmed_topic", "/vision/target_confirmed");
    control_granted_topic_ = declare_parameter<std::string>(
      "control_granted_topic", "/homing/vision_control_granted");
  }

  void declare_search_parameters()
  {
    search_length_ = declare_parameter<double>("arena_length_m", 10.0);
    search_width_ = declare_parameter<double>("arena_width_m", 15.0);
    search_max_depth_ = declare_parameter<double>("arena_depth_m", 11.0);
    arena_start_corner_ = declare_parameter<std::string>(
      "arena_start_corner", "bottom_left");
    horizontal_margin_ = declare_parameter<double>("arena_safety_margin_m", 0.5);
    surface_margin_ = declare_parameter<double>("surface_safety_margin_m", 0.5);
    bottom_margin_ = declare_parameter<double>("bottom_safety_margin_m", 1.0);
    waypoint_tolerance_ = declare_parameter<double>("waypoint_tolerance_m", 0.45);
  }

  void declare_tracking_parameters()
  {
    target_class_id_ = declare_parameter<int>("target_class_id", -1);
    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    if (target_class_id_ >= 0) {
      RCLCPP_WARN(
        get_logger(), "target_class_id is deprecated; using it as buoy_class_id=%d",
        target_class_id_);
      buoy_class_id_ = target_class_id_;
    }
    standoff_distance_ = declare_parameter<double>("standoff_distance_m", 1.5);
    target_recent_sec_ = declare_parameter<double>("target_recent_sec", 0.8);
    visual_bbox_association_ratio_ = declare_parameter<double>(
      "visual_bbox_association_ratio", 0.25);
    stick_pairing_distance_ratio_ = declare_parameter<double>(
      "stick_pairing_distance_ratio", 0.30);
    target_reacquire_timeout_sec_ = declare_parameter<double>(
      "target_reacquire_timeout_sec", 5.0);
  }

  void declare_control_parameters()
  {
    dry_run_ = declare_parameter<bool>("dry_run", false);
    request_vision_mode_ = declare_parameter<bool>("request_vision_mode", true);
    vision_mode_name_ = declare_parameter<std::string>("vision_mode_name", "STABILIZE");
    require_fcu_armed_ = declare_parameter<bool>("require_fcu_armed", true);
    guided_request_period_sec_ = declare_parameter<double>("guided_request_period_sec", 2.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.20);
    handoff_hold_sec_ = declare_parameter<double>("handoff_hold_sec", 0.7);
    max_handoff_speed_ = declare_parameter<double>("max_handoff_speed_mps", 0.20);
    visual_sequence_timeout_sec_ = declare_parameter<double>(
      "visual_sequence_timeout_sec", 35.0);
    acoustic_handoff_enabled_ = declare_parameter<bool>(
      "acoustic_handoff_enabled", true);
  }

  void validate_parameters() const
  {
    if (search_length_ <= 0.0 || search_width_ <= 0.0 || search_max_depth_ <= 0.0) {
      throw std::invalid_argument("search volume dimensions must be positive");
    }
    if (arena_start_frame_topic_.empty() || arena_frame_.empty()) {
      throw std::invalid_argument("arena frame names must not be empty");
    }
    if (
      arena_start_corner_ != "bottom_left" &&
      arena_start_corner_ != "bottom_right")
    {
      throw std::invalid_argument(
              "arena_start_corner must be bottom_left or bottom_right");
    }
    if (
      2.0 * horizontal_margin_ >= std::min(search_length_, search_width_) ||
      surface_margin_ + bottom_margin_ >= search_max_depth_)
    {
      throw std::invalid_argument("safety margins leave no searchable volume");
    }
    if (waypoint_tolerance_ <= 0.0) {
      throw std::invalid_argument("waypoint_tolerance_m must be positive");
    }
    if (
      buoy_class_id_ == stick_class_id_ || visual_bbox_association_ratio_ <= 0.0 ||
      stick_pairing_distance_ratio_ <= 0.0)
    {
      throw std::invalid_argument("buoy/stick class and visual pairing parameters are invalid");
    }
    if (
      odom_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("odometry timing parameters are invalid");
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
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
    odom_frame_ = message->header.frame_id;
    latest_odom_ = *message;
    last_odom_received_ = now();
  }

  void on_arena_start_frame(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Ignored /guided/start_frame with an empty parent frame");
      return;
    }
    if (state_ != MissionState::IDLE && arena_frame_ready_) {
      RCLCPP_WARN(
        get_logger(), "Ignored arena start-frame update while a mission is active");
      return;
    }
    const auto yaw = yaw_from_quaternion(message->pose.orientation);
    const auto & origin = message->pose.position;
    if (
      !yaw || !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
      !std::isfinite(origin.z))
    {
      RCLCPP_WARN(get_logger(), "Ignored invalid /guided/start_frame pose");
      return;
    }
    if (!odom_frame_.empty() && message->header.frame_id != odom_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignored /guided/start_frame in '%s'; odometry uses '%s'",
        message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    arena_parent_frame_ = message->header.frame_id;
    arena_origin_ = origin;
    arena_yaw_rad_ = *yaw;
    arena_frame_ready_ = true;
    publish_arena_tf();
    RCLCPP_INFO(
      get_logger(),
      "Arena frame captured from %s: origin=(%.3f, %.3f, %.3f), yaw=%.2f deg",
      arena_start_frame_topic_.c_str(),
      arena_origin_.x, arena_origin_.y, arena_origin_.z,
      arena_yaw_rad_ * 180.0 / 3.14159265358979323846);
  }

  void on_mission_start(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {
      abort_to_idle("mission start was cleared");
      return;
    }
    if (state_ == MissionState::IDLE) {
      start_mission();
    }
  }

  void on_vision_search(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!acoustic_handoff_enabled_) {
      return;
    }
    vision_search_active_ = message->data;
    if (!message->data) {
      if (!acoustic_control_granted_) {
        target_confirmation_sent_ = false;
      }
      return;
    }

    acoustic_control_granted_ = false;
    target_confirmation_sent_ = perception_target_confirmed_;
    publish_vision_enable(false);
    RCLCPP_INFO(
      get_logger(),
      "[HANDOFF] acoustic controller requested vision target confirmation");
  }

  void on_target_confirmed(const std_msgs::msg::Bool::SharedPtr message)
  {
    perception_target_confirmed_ = message->data;
    if (!acoustic_handoff_enabled_ || !vision_search_active_) {
      return;
    }
    target_confirmation_sent_ = message->data;
    if (message->data) {
      RCLCPP_INFO(
        get_logger(),
        "[HANDOFF] perception pipeline confirmed the buoy target");
    }
  }

  void on_tracks(const TrackArray::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      return;
    }
    if (
      !odom_frame_.empty() && message->header.frame_id != odom_frame_ &&
      message->header.frame_id != arena_parent_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored buoy tracks in '%s'; expected odom frame '%s'",
        message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    for (const auto & input : message->tracks) {
      Point position = input.position_mission;
      if (arena_frame_ready_ && !inside_arena(position, 0.0)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignored odom buoy track outside configured arena: id=%u", input.id);
        continue;
      }
      auto * track = find_track(input.id);
      if (!track) {
        BuoyTrack created;
        created.id = input.id;
        created.class_id = input.class_id;
        created.status = input.status;
        tracks_.push_back(created);
        track = &tracks_.back();
      }
      track->class_id = input.class_id;
      track->position = position;
      track->rms = input.position_rms_m;
      track->inliers = input.observation_count;
      track->confirmed =
        input.status == TrackMessage::CONFIRMED ||
        input.status == TrackMessage::ASSIGNED ||
        input.status == TrackMessage::APPROACHING ||
        input.status == TrackMessage::SERVICED;
      track->last_seen = rclcpp::Time(input.last_seen, RCL_ROS_TIME);
      if (
        track->status == TrackMessage::CANDIDATE ||
        track->status == TrackMessage::CONFIRMED ||
        track->status == TrackMessage::STALE ||
        track->status == TrackMessage::LOST)
      {
        track->status = input.status;
      }
      if (active_target_id_ && input.id == *active_target_id_) {
        active_target_last_seen_ = track->last_seen;
      }
    }
    if (!message->tracks.empty()) {
      last_map_observation_ = now();
    }
  }

  void on_control_granted(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!acoustic_handoff_enabled_) {
      return;
    }
    if (!message->data) {
      if (acoustic_control_granted_) {
        publish_vision_enable(false);
        acoustic_direct_strike_active_ = false;
        RCLCPP_WARN(get_logger(), "[HANDOFF] vision control grant was revoked");
      }
      acoustic_control_granted_ = false;
      return;
    }
    if (!vision_search_active_ || !target_confirmation_sent_) {
      RCLCPP_WARN(
        get_logger(),
        "[HANDOFF] ignored control grant without an active confirmed request");
      return;
    }
    acoustic_control_granted_ = true;
    acoustic_direct_strike_active_ = true;
    active_command_odom_.reset();
    visual_servo_started_ = now();
    publish_vision_enable(true);
    transition_to(
      MissionState::VISUAL_SERVO,
      "acoustic handoff complete; starting RC buoy strike directly");
    RCLCPP_INFO(
      get_logger(),
      "[HANDOFF] acoustic RC stopped; direct visual RC strike is enabled");
  }

  void start_mission()
  {
    if (!latest_odom_ || odom_is_stale()) {
      RCLCPP_WARN(get_logger(), "Mission start rejected: fresh /odometry/filtered is required");
      return;
    }
    if (!arena_frame_ready_) {
      RCLCPP_WARN(
        get_logger(),
        "Mission start rejected: waiting for a valid %s",
        arena_start_frame_topic_.c_str());
      return;
    }
    if (
      !dry_run_ && require_fcu_armed_ &&
      (!fcu_state_ || !fcu_state_->connected || !fcu_state_->armed))
    {
      RCLCPP_ERROR(
        get_logger(), "Mission start rejected: connected and armed FCU is required");
      return;
    }
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const BuoyTrack & track) {
          return !inside_arena(track.position, 0.0);
        }),
      tracks_.end());
    active_target_id_.reset();
    last_target_bbox_center_.reset();
    publish_vision_enable(false);
    transition_to(
      MissionState::WAIT_FOR_TARGET,
      "start flag received; waiting for a confirmed mapped buoy");
  }

  void abort_to_idle(const std::string & reason)
  {
    publish_vision_enable(false);
    if (!acoustic_control_granted_) {
      target_confirmation_sent_ = false;
    }
    active_command_odom_.reset();
    active_target_id_.reset();
    tracks_.clear();
    last_target_bbox_center_.reset();
    transition_to(MissionState::IDLE, reason);
  }

  void on_timer()
  {
    publish_arena_tf();
    if (state_ == MissionState::IDLE || state_ == MissionState::COMPLETE) {
      return;
    }
    if (acoustic_handoff_enabled_ && !acoustic_control_granted_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[HANDOFF] mapping is active, but all vehicle commands are locked "
        "until /homing/vision_control_granted=true");
      return;
    }
    if (state_ != MissionState::FAILSAFE && odom_is_stale()) {
      fail("/odometry/filtered timeout");
      return;
    }
    if (
      state_ != MissionState::FAILSAFE && !dry_run_ && require_fcu_armed_ &&
      (!fcu_state_ || !fcu_state_->connected || !fcu_state_->armed))
    {
      fail("FCU disconnected or disarmed during active mission");
      return;
    }
    if (state_ != MissionState::FAILSAFE) {
      const auto robot = current_odom_position();
      if (robot && !inside_arena(*robot, 0.25)) {
        fail("robot pose left the configured mission volume");
        return;
      }
    }
    switch (state_) {
      case MissionState::WAIT_FOR_TARGET:
        run_wait_for_target();
        break;
      case MissionState::GUIDED_APPROACH:
        run_guided_approach();
        break;
      case MissionState::HANDOFF_WAIT:
        run_handoff();
        break;
      case MissionState::VISUAL_SERVO:
        run_visual_servo_watchdog();
        break;
      case MissionState::FAILSAFE:
      case MissionState::IDLE:
      case MissionState::COMPLETE:
        break;
    }
    publish_active_waypoint();
  }

  void run_wait_for_target()
  {
    const auto robot = current_odom_position();
    if (!robot) {
      return;
    }
    std::optional<uint32_t> selected;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & track : tracks_) {
      if (!track.confirmed || track.status != TrackMessage::CONFIRMED) {
        continue;
      }
      const double distance = point_distance(*robot, track.position);
      if (distance < best_distance) {
        selected = track.id;
        best_distance = distance;
      }
    }
    if (selected) {
      begin_target(*selected, "start flag active and nearest mapped buoy is confirmed");
    }
  }

  void begin_target(uint32_t id, const std::string & reason)
  {
    auto * track = find_track(id);
    const auto robot = current_odom_position();
    if (!track || !robot) {
      return;
    }
    active_target_id_ = id;
    active_target_last_seen_ = track->last_seen;
    last_target_bbox_center_ = track->last_image_center;
    target_started_ = now();
    track->status = TrackMessage::ASSIGNED;
    active_command_odom_ = make_standoff(*robot, track->position);
    if (!active_command_odom_ || !inside_safe_arena(*active_command_odom_)) {
      track->status = TrackMessage::LOST;
      active_target_id_.reset();
      active_command_odom_.reset();
      RCLCPP_WARN(get_logger(), "Buoy id=%u has no safe in-volume standoff waypoint", id);
      return;
    }
    track->status = TrackMessage::APPROACHING;
    transition_to(MissionState::GUIDED_APPROACH, reason);
  }

  std::optional<Point> make_standoff(const Point & robot, const Point & buoy) const
  {
    Point output = buoy;
    const double distance = horizontal_distance(robot, buoy);
    if (distance > 1e-6) {
      output.x -= standoff_distance_ * (buoy.x - robot.x) / distance;
      output.y -= standoff_distance_ * (buoy.y - robot.y) / distance;
    } else {
      output.x -= standoff_distance_;
    }
    Point arena = odom_to_arena(output);
    arena.x = std::clamp(
      arena.x, horizontal_margin_, search_length_ - horizontal_margin_);
    if (arena_start_corner_ == "bottom_left") {
      arena.y = std::clamp(
        arena.y, -search_width_ + horizontal_margin_, -horizontal_margin_);
    } else {
      arena.y = std::clamp(
        arena.y, horizontal_margin_, search_width_ - horizontal_margin_);
    }
    arena.z = std::clamp(
      arena.z, -(search_max_depth_ - bottom_margin_), -surface_margin_);
    return arena_to_odom(arena);
  }

  void run_guided_approach()
  {
    auto * track = active_target();
    const auto robot = current_odom_position();
    if (!track || !robot || !active_command_odom_) {
      recover_after_target("active target or approach waypoint disappeared");
      return;
    }
    track->status = TrackMessage::APPROACHING;
    const bool waypoint_reached =
      point_distance(*robot, *active_command_odom_) <= waypoint_tolerance_;
    const bool close_enough =
      point_distance(*robot, track->position) <= standoff_distance_ + waypoint_tolerance_;
    if (!waypoint_reached && !close_enough) {
      return;
    }
    if (track->confirmed && target_is_recent()) {
      handoff_started_ = now();
      transition_to(MissionState::HANDOFF_WAIT, "standoff reached and target is visible");
      return;
    }
    if ((now() - target_started_).seconds() > target_reacquire_timeout_sec_) {
      track->status = TrackMessage::LOST;
      recover_after_target("target not visible at mapped standoff");
    }
  }

  void run_handoff()
  {
    if (!target_is_recent()) {
      if ((now() - handoff_started_).seconds() > target_reacquire_timeout_sec_) {
        if (auto * track = active_target()) {
          track->status = TrackMessage::LOST;
        }
        recover_after_target("target lost during Guided-to-vision handoff");
      }
      return;
    }
    if (!vision_mode_ready()) {
      return;
    }
    if (current_speed() > max_handoff_speed_) {
      handoff_started_ = now();
      return;
    }
    if ((now() - handoff_started_).seconds() >= handoff_hold_sec_) {
      active_command_odom_.reset();
      publish_vision_enable(true);
      visual_servo_started_ = now();
      transition_to(MissionState::VISUAL_SERVO, "vehicle settled and selected target remained visible");
    }
  }

  void run_visual_servo_watchdog()
  {
    if (acoustic_direct_strike_active_) {
      if (
        (now() - visual_servo_started_).seconds() >
        visual_sequence_timeout_sec_)
      {
        acoustic_direct_strike_active_ = false;
        fail("direct acoustic-to-vision strike exceeded manager timeout");
      }
      return;
    }
    if (protected_visual_sequence()) {
      if ((now() - visual_servo_started_).seconds() > visual_sequence_timeout_sec_) {
        publish_vision_enable(false);
        if (auto * track = active_target()) {
          track->status = TrackMessage::LOST;
        }
        recover_after_target("visual strike sequence exceeded manager timeout");
      }
      return;
    }
    if (!target_is_recent()) {
      publish_vision_enable(false);
      if (auto * track = active_target()) {
        track->status = TrackMessage::LOST;
      }
      recover_after_target("selected target timed out during visual servo");
    }
  }

  void on_target_complete(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data || state_ != MissionState::VISUAL_SERVO) {
      return;
    }
    publish_vision_enable(false);
    if (acoustic_direct_strike_active_) {
      acoustic_direct_strike_active_ = false;
      transition_to(
        MissionState::COMPLETE,
        "direct acoustic-to-vision buoy strike completed");
      return;
    }
    if (auto * track = active_target()) {
      track->status = TrackMessage::SERVICED;
      RCLCPP_INFO(get_logger(), "Buoy id=%u marked SERVICED", track->id);
    }
    active_target_id_.reset();
    active_command_odom_.reset();
    transition_to(MissionState::COMPLETE, "flag-started buoy strike completed");
  }

  void on_target_failed(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data || state_ != MissionState::VISUAL_SERVO) {
      return;
    }
    publish_vision_enable(false);
    if (acoustic_direct_strike_active_) {
      acoustic_direct_strike_active_ = false;
      fail("direct visual strike controller reported target failure");
      return;
    }
    if (auto * track = active_target()) {
      track->status = TrackMessage::LOST;
    }
    fail("visual strike controller reported target failure");
  }

  void recover_after_target(const std::string & reason)
  {
    publish_vision_enable(false);
    if (auto * track = active_target()) {
      track->status = TrackMessage::LOST;
    }
    active_target_id_.reset();
    active_command_odom_.reset();
    transition_to(MissionState::WAIT_FOR_TARGET, reason);
  }

  bool vision_mode_ready()
  {
    if (acoustic_handoff_enabled_ && !acoustic_control_granted_) {
      return false;
    }
    if (!request_vision_mode_ || dry_run_) {
      return true;
    }
    if (!fcu_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for MAVROS state before vision handoff");
      return false;
    }
    if (fcu_state_->mode == vision_mode_name_) {
      return true;
    }
    request_mode(vision_mode_name_);
    return false;
  }

  void request_mode(const std::string & mode)
  {
    if (
      (acoustic_handoff_enabled_ && !acoustic_control_granted_) ||
      dry_run_ || mode_request_pending_ ||
      (now() - last_mode_request_).seconds() < guided_request_period_sec_)
    {
      return;
    }
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MAVROS set_mode service is unavailable");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    mode_request_pending_ = true;
    last_mode_request_ = now();
    set_mode_client_->async_send_request(
      request,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        mode_request_pending_ = false;
        if (!future.get()->mode_sent) {
          RCLCPP_WARN(get_logger(), "Pixhawk rejected mode request '%s'", mode.c_str());
        }
      });
  }

  void publish_active_waypoint()
  {
    if (!active_command_odom_ || !arena_frame_ready_) {
      return;
    }
    geometry_msgs::msg::PointStamped debug;
    debug.header.stamp = now();
    debug.header.frame_id = odom_frame_;
    debug.point = *active_command_odom_;
    waypoint_debug_pub_->publish(debug);

    if (!latest_odom_) {
      return;
    }

    mavros_msgs::msg::PositionTarget command;
    command.header.stamp = debug.header.stamp;
    command.header.frame_id = odom_frame_;
    command.coordinate_frame =
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    command.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    // Keep mapping and control in the same odom-fixed ROS ENU frame. Only the
    // absolute position is commanded; yaw and yaw rate are left uncontrolled.
    command.position = *active_command_odom_;
    if ((acoustic_handoff_enabled_ && !acoustic_control_granted_) || dry_run_) {
      return;
    }
    waypoint_pub_->publish(command);
  }

  void on_detection_2d(const Detection2D::SharedPtr detection)
  {
    const bool direct_handoff_detection =
      acoustic_handoff_enabled_ && vision_search_active_;
    if (!active_target_id_ && !direct_handoff_detection) {
      pending_detection_frames_.clear();
      return;
    }
    const uint64_t frame_key =
      (static_cast<uint64_t>(static_cast<uint32_t>(detection->header.stamp.sec)) << 32) |
      detection->header.stamp.nanosec;
    auto & frame = pending_detection_frames_[frame_key];
    frame.expected = detection->detections_in_frame;
    if (detection->detected) {
      const auto duplicate = std::find_if(
        frame.detections.begin(), frame.detections.end(),
        [detection](const auto & item) {
          return item.detection_index == detection->detection_index;
        });
      if (duplicate == frame.detections.end()) {
        frame.detections.push_back(*detection);
      }
    }
    const bool complete =
      (frame.expected == 0 && !detection->detected) ||
      (frame.expected > 0 && frame.detections.size() >= frame.expected);
    if (!complete) {
      return;
    }
    const auto detections = frame.detections;
    pending_detection_frames_.erase(frame_key);
    while (pending_detection_frames_.size() > 60) {
      pending_detection_frames_.erase(pending_detection_frames_.begin());
    }
    publish_selected_detection_pair(detections);
  }

  template<typename DetectionT>
  static std::pair<double, double> normalized_bbox_center(const DetectionT & detection)
  {
    return {
      detection.center_x / std::max(1u, detection.image_width),
      detection.center_y / std::max(1u, detection.image_height)};
  }

  void publish_selected_detection_pair(const std::vector<Detection2D> & detections)
  {
    auto * track = active_target();
    if (!track && !(acoustic_handoff_enabled_ && vision_search_active_)) {
      return;
    }
    const Detection2D * selected_buoy = nullptr;
    double best_buoy_score = std::numeric_limits<double>::infinity();
    for (const auto & detection : detections) {
      if (
        !detection.detected || detection.class_id != buoy_class_id_ ||
        detection.image_width == 0 || detection.image_height == 0)
      {
        continue;
      }
      const auto center = normalized_bbox_center(detection);
      if (last_target_bbox_center_) {
        const double distance = std::hypot(
          center.first - last_target_bbox_center_->first,
          center.second - last_target_bbox_center_->second);
        if (distance <= visual_bbox_association_ratio_ && distance < best_buoy_score) {
          selected_buoy = &detection;
          best_buoy_score = distance;
        }
      } else {
        const double area = static_cast<double>(detection.width) * detection.height;
        const double score = -area;
        if (score < best_buoy_score) {
          selected_buoy = &detection;
          best_buoy_score = score;
        }
      }
    }
    if (!selected_buoy) {
      return;
    }
    const auto buoy_center = normalized_bbox_center(*selected_buoy);
    const Detection2D * selected_stick = nullptr;
    double best_stick_distance = stick_pairing_distance_ratio_;
    for (const auto & detection : detections) {
      if (
        !detection.detected || detection.class_id != stick_class_id_ ||
        detection.image_width == 0 || detection.image_height == 0)
      {
        continue;
      }
      const auto center = normalized_bbox_center(detection);
      const double distance = std::hypot(
        center.first - buoy_center.first, center.second - buoy_center.second);
      if (distance <= best_stick_distance) {
        selected_stick = &detection;
        best_stick_distance = distance;
      }
    }

    std_msgs::msg::Float32MultiArray output;
    append_bbox_block(output, *selected_buoy);
    if (selected_stick) {
      append_bbox_block(output, *selected_stick);
    }
    selected_bbox_pub_->publish(output);
    last_target_bbox_center_ = buoy_center;
    if (track) {
      track->last_image_center = buoy_center;
    }
    active_target_last_seen_ = now();
  }

  static void append_bbox_block(
    std_msgs::msg::Float32MultiArray & output, const Detection2D & detection)
  {
    const double stamp = static_cast<double>(detection.header.stamp.sec) +
      static_cast<double>(detection.header.stamp.nanosec) * 1e-9;
    const std::array<float, 10> block = {
      static_cast<float>(stamp), 1.0F, static_cast<float>(detection.class_id),
      detection.confidence, detection.center_x, detection.center_y,
      detection.width, detection.height, static_cast<float>(detection.image_width),
      static_cast<float>(detection.image_height)};
    output.data.insert(output.data.end(), block.begin(), block.end());
  }

  void publish_vision_enable(bool enabled)
  {
    std_msgs::msg::Bool output;
    output.data = enabled;
    vision_enable_pub_->publish(output);
  }

  void transition_to(MissionState next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Mission state %s -> %s: %s",
      state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    publish_state();
  }

  void publish_state()
  {
    std_msgs::msg::String output;
    output.data = state_name(state_);
    state_pub_->publish(output);
  }

  void fail(const std::string & reason)
  {
    publish_vision_enable(false);
    active_command_odom_.reset();
    transition_to(MissionState::FAILSAFE, reason);
  }

  static const char * state_name(MissionState state)
  {
    switch (state) {
      case MissionState::IDLE: return "IDLE";
      case MissionState::WAIT_FOR_TARGET: return "WAIT_FOR_TARGET";
      case MissionState::GUIDED_APPROACH: return "GUIDED_APPROACH";
      case MissionState::HANDOFF_WAIT: return "HANDOFF_WAIT";
      case MissionState::VISUAL_SERVO: return "VISUAL_SERVO";
      case MissionState::COMPLETE: return "COMPLETE";
      case MissionState::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  bool odom_is_stale() const
  {
    return !latest_odom_ || (now() - last_odom_received_).seconds() > odom_timeout_sec_;
  }

  std::optional<Point> current_odom_position() const
  {
    if (!latest_odom_ || !arena_frame_ready_) {
      return std::nullopt;
    }
    return latest_odom_->pose.pose.position;
  }

  double current_speed() const
  {
    if (!latest_odom_) {
      return std::numeric_limits<double>::infinity();
    }
    const auto & velocity = latest_odom_->twist.twist.linear;
    return std::hypot(std::hypot(velocity.x, velocity.y), velocity.z);
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

  Point arena_to_odom(const Point & arena) const
  {
    const double cosine = std::cos(arena_yaw_rad_);
    const double sine = std::sin(arena_yaw_rad_);
    Point output;
    output.x = arena_origin_.x + cosine * arena.x - sine * arena.y;
    output.y = arena_origin_.y + sine * arena.x + cosine * arena.y;
    output.z = arena_origin_.z + arena.z;
    return output;
  }

  bool inside_arena(const Point & odom, double tolerance) const
  {
    if (!arena_frame_ready_) {
      return false;
    }
    const Point arena = odom_to_arena(odom);
    const double y_min =
      arena_start_corner_ == "bottom_left" ? -search_width_ : 0.0;
    const double y_max =
      arena_start_corner_ == "bottom_left" ? 0.0 : search_width_;
    return
      arena.x >= -tolerance && arena.x <= search_length_ + tolerance &&
      arena.y >= y_min - tolerance && arena.y <= y_max + tolerance &&
      arena.z >= -search_max_depth_ - tolerance && arena.z <= tolerance;
  }

  bool inside_safe_arena(const Point & odom) const
  {
    if (!arena_frame_ready_) {
      return false;
    }
    const Point arena = odom_to_arena(odom);
    const double y_min = arena_start_corner_ == "bottom_left" ?
      -search_width_ + horizontal_margin_ : horizontal_margin_;
    const double y_max = arena_start_corner_ == "bottom_left" ?
      -horizontal_margin_ : search_width_ - horizontal_margin_;
    return
      arena.x >= horizontal_margin_ &&
      arena.x <= search_length_ - horizontal_margin_ &&
      arena.y >= y_min && arena.y <= y_max &&
      arena.z >= -(search_max_depth_ - bottom_margin_) &&
      arena.z <= -surface_margin_;
  }

  void publish_arena_tf()
  {
    if (!arena_frame_ready_ || !arena_tf_broadcaster_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = arena_parent_frame_;
    transform.child_frame_id = arena_frame_;
    transform.transform.translation.x = arena_origin_.x;
    transform.transform.translation.y = arena_origin_.y;
    transform.transform.translation.z = arena_origin_.z;
    transform.transform.rotation.z = std::sin(0.5 * arena_yaw_rad_);
    transform.transform.rotation.w = std::cos(0.5 * arena_yaw_rad_);
    arena_tf_broadcaster_->sendTransform(transform);
  }

  BuoyTrack * find_track(uint32_t id)
  {
    const auto result = std::find_if(
      tracks_.begin(), tracks_.end(), [id](const auto & track) {return track.id == id;});
    return result == tracks_.end() ? nullptr : &*result;
  }

  BuoyTrack * active_target()
  {
    return active_target_id_ ? find_track(*active_target_id_) : nullptr;
  }

  bool target_is_recent() const
  {
    return active_target_id_ &&
      (now() - active_target_last_seen_).seconds() <= target_recent_sec_;
  }

  bool protected_visual_sequence() const
  {
    return visual_controller_state_ == "INSERT_FORK" ||
           visual_controller_state_ == "STRIKE" ||
           visual_controller_state_ == "RETRACT" ||
           visual_controller_state_ == "VERIFY_RELEASE";
  }

  // Parameters and topic names.
  std::string odom_topic_, arena_start_frame_topic_, arena_frame_;
  std::string detection_2d_topic_, mission_start_topic_;
  std::string target_complete_topic_, target_failed_topic_;
  std::string mission_state_topic_, buoy_tracks_topic_;
  std::string selected_bbox_topic_, vision_enable_topic_;
  std::string waypoint_debug_topic_;
  std::string waypoint_topic_;
  std::string fcu_state_topic_, set_mode_service_, visual_state_topic_;
  std::string vision_search_topic_, target_confirmed_topic_, control_granted_topic_;
  std::string vision_mode_name_;
  std::string arena_start_corner_;
  std::string odom_frame_, arena_parent_frame_;
  double search_length_, search_width_, search_max_depth_;
  Point arena_origin_;
  double arena_yaw_rad_{0.0};
  double horizontal_margin_, surface_margin_, bottom_margin_;
  double waypoint_tolerance_;
  double standoff_distance_, target_recent_sec_;
  double visual_bbox_association_ratio_, stick_pairing_distance_ratio_;
  double target_reacquire_timeout_sec_;
  double guided_request_period_sec_;
  double odom_timeout_sec_;
  double handoff_hold_sec_, max_handoff_speed_, visual_sequence_timeout_sec_;
  int target_class_id_, buoy_class_id_, stick_class_id_;
  bool dry_run_, request_vision_mode_, require_fcu_armed_;
  bool acoustic_handoff_enabled_{true};

  MissionState state_{MissionState::IDLE};
  bool arena_frame_ready_{false};
  bool mode_request_pending_{false};
  bool vision_search_active_{false};
  bool perception_target_confirmed_{false};
  bool target_confirmation_sent_{false};
  bool acoustic_control_granted_{false};
  bool acoustic_direct_strike_active_{false};
  std::optional<Point> active_command_odom_;
  std::vector<BuoyTrack> tracks_;
  std::optional<uint32_t> active_target_id_;
  std::optional<std::pair<double, double>> last_target_bbox_center_;
  std::map<uint64_t, DetectionFrame> pending_detection_frames_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<mavros_msgs::msg::State> fcu_state_;
  std::string visual_controller_state_{"IDLE"};
  rclcpp::Time last_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_observation_{0, 0, RCL_ROS_TIME};
  rclcpp::Time active_target_last_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time handoff_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time visual_servo_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mode_request_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arena_start_frame_sub_;
  rclcpp::Subscription<Detection2D>::SharedPtr detection_2d_sub_;
  rclcpp::Subscription<TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_failed_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fcu_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_search_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_granted_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr selected_bbox_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_enable_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_debug_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr waypoint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> arena_tf_broadcaster_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_vision_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<auv_vision_nav_control::BuoyMissionManager>());
  rclcpp::shutdown();
  return 0;
}
