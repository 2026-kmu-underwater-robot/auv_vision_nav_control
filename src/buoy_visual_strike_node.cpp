#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace auv_vision_nav_control
{

class BuoyVisualStrikeNode : public rclcpp::Node
{
public:
  BuoyVisualStrikeNode()
  : Node("buoy_visual_strike_node")
  {
    declare_parameters();
    validate_parameters();

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 20,
      std::bind(&BuoyVisualStrikeNode::on_bbox, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyVisualStrikeNode::on_enable, this, std::placeholders::_1));
    control_granted_sub_ = create_subscription<std_msgs::msg::Bool>(
      control_granted_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&BuoyVisualStrikeNode::on_control_granted, this, std::placeholders::_1));
    if (!depth_pose_topic_.empty()) {
      depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        depth_pose_topic_, 20,
        std::bind(&BuoyVisualStrikeNode::on_depth, this, std::placeholders::_1));
    }
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    rc_monitor_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_monitor_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      visual_state_topic_, rclcpp::QoS(1).reliable().transient_local());
    complete_pub_ = create_publisher<std_msgs::msg::Bool>(target_complete_topic_, 10);
    failed_pub_ = create_publisher<std_msgs::msg::Bool>(target_failed_topic_, 10);

    const double period = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&BuoyVisualStrikeNode::on_timer, this));
    state_entered_at_ = now();
    publish_state();
    publish_result(false, false);
    RCLCPP_INFO(
      get_logger(),
      "Visual strike ready: bbox=%s enable=%s RC=%s dry_run=%s classes buoy=%d stick=%d",
      bbox_topic_.c_str(), enable_topic_.c_str(), rc_override_topic_.c_str(),
      dry_run_ ? "true" : "false", buoy_class_id_, stick_class_id_);
  }

  void publish_release_once()
  {
    auto channels = nochange_channels();
    release_channels(channels);
    publish_channels(channels, true);
  }

private:
  enum class State
  {
    IDLE,
    ACQUIRE_BUOY,
    ALIGN_STICK,
    INSERT_FORK,
    STRIKE,
    RETRACT,
    VERIFY_RELEASE,
    COMPLETE,
    FAILSAFE
  };

  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{0};
  };

  void declare_parameters()
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/mission/control_enable");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    visual_state_topic_ = declare_parameter<std::string>(
      "visual_state_topic", "/mission/visual_state");
    target_complete_topic_ = declare_parameter<std::string>(
      "target_complete_topic", "/mission/target_complete");
    target_failed_topic_ = declare_parameter<std::string>(
      "target_failed_topic", "/mission/target_failed");
    rc_override_topic_ = declare_parameter<std::string>(
      "rc_override_topic", "/mavros/rc/override");
    rc_monitor_topic_ = declare_parameter<std::string>(
      "rc_monitor_topic", "/mission/rc_command");
    control_granted_topic_ = declare_parameter<std::string>(
      "control_granted_topic", "/homing/vision_control_granted");

    dry_run_ = declare_parameter<bool>("dry_run", false);
    acoustic_handoff_enabled_ = declare_parameter<bool>(
      "acoustic_handoff_enabled", true);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 0.7);
    acquire_timeout_sec_ = declare_parameter<double>("acquire_timeout_sec", 8.0);
    operation_timeout_sec_ = declare_parameter<double>("operation_timeout_sec", 25.0);
    lpf_tau_sec_ = declare_parameter<double>("lpf_tau_sec", 0.25);

    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    min_detection_hits_ = declare_parameter<int>("min_detection_hits", 3);
    approach_area_ratio_ = declare_parameter<double>("approach_area_ratio", 0.30);
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1650);
    approach_forward_min_pwm_ = declare_parameter<int>("approach_forward_min_pwm", 1560);

    fork_target_x_ = declare_parameter<double>("fork_target_x", 0.30);
    fork_target_y_ = declare_parameter<double>("fork_target_y", 0.70);
    stick_deadband_x_ = declare_parameter<double>("stick_deadband_x", 0.06);
    stick_deadband_y_ = declare_parameter<double>("stick_deadband_y", 0.08);
    align_stable_sec_ = declare_parameter<double>("align_stable_sec", 0.7);

    insert_pwm_ = declare_parameter<int>("insert_pwm", 1560);
    insert_duration_sec_ = declare_parameter<double>("insert_duration_sec", 0.8);
    strike_pwm_ = declare_parameter<int>("strike_pwm", 1620);
    strike_duration_sec_ = declare_parameter<double>("strike_duration_sec", 0.3);
    retract_pwm_ = declare_parameter<int>("retract_pwm", 1420);
    retract_duration_sec_ = declare_parameter<double>("retract_duration_sec", 0.5);
    verify_clear_sec_ = declare_parameter<double>("verify_clear_sec", 1.0);
    verify_timeout_sec_ = declare_parameter<double>("verify_timeout_sec", 3.0);
    release_area_ratio_ = declare_parameter<double>("release_area_ratio", 0.65);
    max_target_retries_ = declare_parameter<int>("max_target_retries", 2);

    require_depth_ = declare_parameter<bool>("require_depth", true);
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", -1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 1.0);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 11.0);
    depth_kp_pwm_per_m_ = declare_parameter<double>("depth_kp_pwm_per_m", 45.0);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 100);
    buoyancy_hold_delta_pwm_ = declare_parameter<int>("buoyancy_hold_delta_pwm", 40);
    vision_throttle_weight_ = declare_parameter<double>("vision_throttle_weight", 0.4);

    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    max_yaw_delta_ = declare_parameter<int>("max_yaw_delta", 180);
    max_tracking_depth_delta_ = declare_parameter<int>("max_tracking_depth_delta", 100);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);
    vertical_positive_is_up_ = declare_parameter<bool>("vertical_positive_is_up", true);
  }

  void validate_parameters() const
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument("PWM range must remain inside 1300..1700");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("RC channel must be in [1,18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("controlled RC channels must be unique");
    }
    if (buoy_class_id_ == stick_class_id_ || min_detection_hits_ < 1) {
      throw std::invalid_argument("buoy/stick class IDs and hit threshold are invalid");
    }
    if (
      approach_area_ratio_ <= 0.0 || approach_area_ratio_ >= 1.0 ||
      release_area_ratio_ <= 0.0 || release_area_ratio_ >= 1.0 ||
      vision_throttle_weight_ < 0.0 || vision_throttle_weight_ > 1.0)
    {
      throw std::invalid_argument("vision ratios must be inside their valid ranges");
    }
    if (
      insert_duration_sec_ <= 0.0 || strike_duration_sec_ <= 0.0 ||
      retract_duration_sec_ <= 0.0 || verify_timeout_sec_ <= 0.0 ||
      max_target_retries_ < 0)
    {
      throw std::invalid_argument("strike sequence timing is invalid");
    }
  }

  void on_enable(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (message->data && !enabled_) {
      if (acoustic_handoff_enabled_ && !control_granted_) {
        RCLCPP_WARN(
          get_logger(),
          "[HANDOFF] ignored visual enable before acoustic control grant");
        return;
      }
      begin_visual_control("mission manager granted visual control");
    } else if (!message->data && enabled_) {
      enabled_ = false;
      buoy_.reset();
      stick_.reset();
      hold_depth_m_.reset();
      transition_to(State::IDLE, "mission manager revoked visual control");
    }
  }

  void begin_visual_control(const std::string & reason)
  {
    enabled_ = true;
    target_retries_ = 0;
    operation_started_at_ = now();
    prestrike_buoy_area_ = 0.0;
    publish_result(false, false);
    if (!require_depth_ || has_recent_depth()) {
      if (depth_m_) {
        hold_depth_m_ = *depth_m_;
      }
      transition_to(State::ACQUIRE_BUOY, reason);
    }
  }

  void on_control_granted(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!acoustic_handoff_enabled_) {
      return;
    }
    control_granted_ = message->data;
    if (control_granted_ && !enabled_) {
      begin_visual_control(
        "acoustic handoff granted; starting RC buoy acquisition");
    }
    if (!control_granted_ && enabled_) {
      enabled_ = false;
      buoy_.reset();
      stick_.reset();
      hold_depth_m_.reset();
      transition_to(State::IDLE, "acoustic controller revoked RC ownership");
    }
    RCLCPP_INFO(
      get_logger(), "[HANDOFF] visual RC ownership=%s",
      control_granted_ ? "granted" : "locked");
  }

  void on_depth(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    const double depth =
      depth_pose_scale_ * message->pose.pose.position.z + depth_pose_offset_m_;
    if (!std::isfinite(depth) || depth < 0.0) {
      return;
    }
    depth_m_ = depth;
    depth_received_at_ = now();
  }

  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() < 10) {
      return;
    }
    std::optional<Detection> buoy;
    std::optional<Detection> stick;
    for (size_t base = 0; base + 9 < message->data.size(); base += 10) {
      bool finite = true;
      for (size_t offset = 0; offset < 10; ++offset) {
        finite = finite && std::isfinite(message->data[base + offset]);
      }
      if (
        !finite || message->data[base + 1] < 0.5F ||
        message->data[base + 8] <= 0.0F || message->data[base + 9] <= 0.0F)
      {
        continue;
      }
      Detection detection{
        message->data[base + 3], message->data[base + 4], message->data[base + 5],
        message->data[base + 6], message->data[base + 7], message->data[base + 8],
        message->data[base + 9], now(), 1};
      const int class_id = static_cast<int>(std::lround(message->data[base + 2]));
      if (
        class_id == buoy_class_id_ &&
        (!buoy || detection.confidence > buoy->confidence))
      {
        buoy = detection;
      } else if (
        class_id == stick_class_id_ &&
        (!stick || detection.confidence > stick->confidence))
      {
        stick = detection;
      }
    }
    if (buoy) {
      update_detection(buoy_, *buoy);
    }
    if (stick) {
      update_detection(stick_, *stick);
    }
  }

  void update_detection(std::optional<Detection> & slot, Detection incoming)
  {
    int hits = 1;
    if (slot && recent(slot)) {
      const double dt = (now() - slot->received_at).seconds();
      hits = slot->consecutive_hits + 1;
      incoming.center_x = static_cast<float>(low_pass(slot->center_x, incoming.center_x, dt));
      incoming.center_y = static_cast<float>(low_pass(slot->center_y, incoming.center_y, dt));
      incoming.width = static_cast<float>(low_pass(slot->width, incoming.width, dt));
      incoming.height = static_cast<float>(low_pass(slot->height, incoming.height, dt));
    }
    incoming.received_at = now();
    incoming.consecutive_hits = hits;
    slot = incoming;
  }

  double low_pass(double previous, double sample, double dt) const
  {
    if (lpf_tau_sec_ <= 1e-9 || dt <= 0.0) {
      return sample;
    }
    const double alpha = dt / (lpf_tau_sec_ + dt);
    return alpha * sample + (1.0 - alpha) * previous;
  }

  void on_timer()
  {
    auto channels = nochange_channels();
    if (!enabled_) {
      release_channels(channels);
      publish_channels(channels);
      return;
    }
    if (state_ == State::IDLE) {
      if (require_depth_ && !has_recent_depth()) {
        release_channels(channels);
        publish_channels(channels);
        return;
      }
      if (depth_m_) {
        hold_depth_m_ = *depth_m_;
      }
      operation_started_at_ = now();
      transition_to(State::ACQUIRE_BUOY, "fresh depth received");
    }
    if (require_depth_ && state_requires_depth() && !has_recent_depth()) {
      fail("depth input timed out");
    }
    if (depth_m_ && *depth_m_ > max_depth_m_) {
      fail("maximum depth exceeded");
    }
    if (
      state_ != State::COMPLETE && state_ != State::FAILSAFE &&
      (now() - operation_started_at_).seconds() > operation_timeout_sec_)
    {
      report_failure("visual strike operation timed out");
    }

    switch (state_) {
      case State::IDLE:
        release_channels(channels);
        break;
      case State::ACQUIRE_BUOY:
        run_acquire(channels);
        break;
      case State::ALIGN_STICK:
        run_align(channels);
        break;
      case State::INSERT_FORK:
        set_neutral(channels);
        hold_depth(channels);
        set_channel(channels, forward_channel_, insert_pwm_);
        if (state_age() >= insert_duration_sec_) {
          transition_to(State::STRIKE, "fork insertion pulse complete");
        }
        break;
      case State::STRIKE:
        set_neutral(channels);
        hold_depth(channels);
        set_channel(channels, forward_channel_, strike_pwm_);
        if (state_age() >= strike_duration_sec_) {
          transition_to(State::RETRACT, "strike pulse complete");
        }
        break;
      case State::RETRACT:
        set_neutral(channels);
        hold_depth(channels);
        set_channel(channels, forward_channel_, retract_pwm_);
        if (state_age() >= retract_duration_sec_) {
          transition_to(State::VERIFY_RELEASE, "retract pulse complete");
        }
        break;
      case State::VERIFY_RELEASE:
        run_verify(channels);
        break;
      case State::COMPLETE:
      case State::FAILSAFE:
        release_channels(channels);
        break;
    }
    publish_channels(channels);
  }

  void run_acquire(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      set_neutral(channels);
      hold_depth(channels);
      if (state_age() >= acquire_timeout_sec_) {
        report_failure("selected buoy was not reacquired");
      }
      return;
    }
    apply_visual_tracking(
      channels, *buoy_, 0.5, 0.5, approach_forward_from_area(*buoy_));
    if (
      buoy_->consecutive_hits >= min_detection_hits_ &&
      area_ratio(*buoy_) >= approach_area_ratio_ &&
      recent(stick_) && stick_->consecutive_hits >= min_detection_hits_)
    {
      transition_to(State::ALIGN_STICK, "close buoy and paired stick are stable");
    }
  }

  void run_align(std::array<uint16_t, 18> & channels)
  {
    if (!recent(stick_)) {
      set_neutral(channels);
      hold_depth(channels);
      transition_to(State::ACQUIRE_BUOY, "paired stick was lost");
      return;
    }
    apply_visual_tracking(
      channels, *stick_, fork_target_x_, fork_target_y_, neutral_pwm_);
    const auto [error_x, error_y] = normalized_error(
      *stick_, fork_target_x_, fork_target_y_);
    if (
      std::abs(error_x) <= stick_deadband_x_ &&
      std::abs(error_y) <= stick_deadband_y_)
    {
      if (!condition_started_at_) {
        condition_started_at_ = now();
      } else if ((now() - *condition_started_at_).seconds() >= align_stable_sec_) {
        prestrike_buoy_area_ = recent(buoy_) ? area_ratio(*buoy_) : approach_area_ratio_;
        transition_to(State::INSERT_FORK, "stick alignment held inside deadband");
      }
    } else {
      condition_started_at_.reset();
    }
  }

  void run_verify(std::array<uint16_t, 18> & channels)
  {
    set_neutral(channels);
    hold_depth(channels);
    const bool absent = !recent(buoy_);
    const bool sufficiently_smaller =
      recent(buoy_) && prestrike_buoy_area_ > 1e-6 &&
      area_ratio(*buoy_) <= prestrike_buoy_area_ * release_area_ratio_;
    if (absent || sufficiently_smaller) {
      if (!condition_started_at_) {
        condition_started_at_ = now();
      } else if ((now() - *condition_started_at_).seconds() >= verify_clear_sec_) {
        transition_to(State::COMPLETE, "buoy cleared the fork after retract");
        publish_result(true, false);
      }
      return;
    }
    condition_started_at_.reset();
    if (state_age() >= verify_timeout_sec_) {
      if (target_retries_ < max_target_retries_) {
        ++target_retries_;
        transition_to(
          recent(stick_) ? State::ALIGN_STICK : State::ACQUIRE_BUOY,
          "buoy remains near fork; retrying strike sequence");
      } else {
        report_failure("buoy remained caught after maximum retries");
      }
    }
  }

  int approach_forward_from_area(const Detection & buoy) const
  {
    const double error = std::max(0.0, approach_area_ratio_ - area_ratio(buoy));
    const int low = std::min(approach_forward_min_pwm_, approach_forward_pwm_);
    const int span = std::max(0, approach_forward_pwm_ - low);
    const double gain = span / approach_area_ratio_;
    return std::clamp(
      low + static_cast<int>(std::lround(gain * error)), low, approach_forward_pwm_);
  }

  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    double target_x, double target_y, int forward_pwm)
  {
    set_neutral(channels);
    const auto [error_x, error_y] = normalized_error(detection, target_x, target_y);
    const double yaw_sign = yaw_invert_ ? -1.0 : 1.0;
    const double vertical_sign = vertical_positive_is_up_ ? -1.0 : 1.0;
    set_channel(
      channels, yaw_channel_,
      neutral_pwm_ + static_cast<int>(yaw_sign * error_x * max_yaw_delta_));
    const int visual_throttle =
      neutral_pwm_ + static_cast<int>(vertical_sign * error_y * max_tracking_depth_delta_);
    int throttle = visual_throttle;
    if (hold_depth_m_ && depth_m_) {
      const int depth_throttle = depth_control_pwm(*hold_depth_m_);
      throttle = static_cast<int>(std::lround(
        vision_throttle_weight_ * visual_throttle +
        (1.0 - vision_throttle_weight_) * depth_throttle));
    }
    set_channel(channels, throttle_channel_, throttle);
    set_channel(channels, forward_channel_, forward_pwm);
  }

  std::pair<double, double> normalized_error(
    const Detection & detection, double target_x, double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp((x - target_x) * 2.0, -1.0, 1.0),
      std::clamp((y - target_y) * 2.0, -1.0, 1.0)};
  }

  double area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  bool recent(const std::optional<Detection> & detection) const
  {
    return detection &&
      (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  bool has_recent_depth() const
  {
    return depth_m_ &&
      (now() - depth_received_at_).seconds() <= depth_timeout_sec_;
  }

  bool state_requires_depth() const
  {
    return state_ != State::IDLE && state_ != State::COMPLETE &&
           state_ != State::FAILSAFE;
  }

  void hold_depth(std::array<uint16_t, 18> & channels) const
  {
    if (hold_depth_m_ && depth_m_) {
      set_channel(channels, throttle_channel_, depth_control_pwm(*hold_depth_m_));
    }
  }

  int depth_control_pwm(double target_depth) const
  {
    const double error = target_depth - *depth_m_;
    int delta = static_cast<int>(std::lround(error * depth_kp_pwm_per_m_));
    delta += buoyancy_hold_delta_pwm_;
    delta = std::clamp(delta, -max_depth_delta_pwm_, max_depth_delta_pwm_);
    const int sign = vertical_positive_is_up_ ? -1 : 1;
    return neutral_pwm_ + sign * delta;
  }

  void report_failure(const std::string & reason)
  {
    transition_to(State::FAILSAFE, reason);
    publish_result(false, true);
  }

  void fail(const std::string & reason)
  {
    transition_to(State::FAILSAFE, reason);
    publish_result(false, true);
  }

  void publish_result(bool complete, bool failed)
  {
    std_msgs::msg::Bool message;
    message.data = complete;
    complete_pub_->publish(message);
    message.data = failed;
    failed_pub_->publish(message);
  }

  void transition_to(State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Visual strike %s -> %s: %s",
      state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    condition_started_at_.reset();
    publish_state();
  }

  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = state_name(state_);
    state_pub_->publish(message);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::ACQUIRE_BUOY: return "ACQUIRE_BUOY";
      case State::ALIGN_STICK: return "ALIGN_STICK";
      case State::INSERT_FORK: return "INSERT_FORK";
      case State::STRIKE: return "STRIKE";
      case State::RETRACT: return "RETRACT";
      case State::VERIFY_RELEASE: return "VERIFY_RELEASE";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  double state_age() const
  {
    return (now() - state_entered_at_).seconds();
  }

  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
  }

  void release_channels(std::array<uint16_t, 18> & channels) const
  {
    set_channel(
      channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(
    std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(
    const std::array<uint16_t, 18> & channels, bool force_release = false)
  {
    mavros_msgs::msg::OverrideRCIn message;
    message.channels = channels;
    rc_monitor_pub_->publish(message);
    if (acoustic_handoff_enabled_ && !control_granted_) {
      return;
    }
    if (!dry_run_ || force_release) {
      rc_pub_->publish(message);
    }
  }

  std::string bbox_topic_, enable_topic_, depth_pose_topic_, visual_state_topic_;
  std::string target_complete_topic_, target_failed_topic_;
  std::string rc_override_topic_, rc_monitor_topic_, control_granted_topic_;
  bool dry_run_{false}, require_depth_{true}, acoustic_handoff_enabled_{true};
  double control_rate_hz_{20.0}, detection_timeout_sec_{0.7};
  double acquire_timeout_sec_{8.0}, operation_timeout_sec_{25.0}, lpf_tau_sec_{0.25};
  int buoy_class_id_{0}, stick_class_id_{1}, min_detection_hits_{3};
  double approach_area_ratio_{0.30};
  int approach_forward_pwm_{1650}, approach_forward_min_pwm_{1560};
  double fork_target_x_{0.30}, fork_target_y_{0.70};
  double stick_deadband_x_{0.06}, stick_deadband_y_{0.08}, align_stable_sec_{0.7};
  int insert_pwm_{1560}, strike_pwm_{1620}, retract_pwm_{1420};
  double insert_duration_sec_{0.8}, strike_duration_sec_{0.3}, retract_duration_sec_{0.5};
  double verify_clear_sec_{1.0}, verify_timeout_sec_{3.0}, release_area_ratio_{0.65};
  int max_target_retries_{2};
  double depth_pose_scale_{-1.0}, depth_pose_offset_m_{0.0}, depth_timeout_sec_{1.0};
  double max_depth_m_{11.0}, depth_kp_pwm_per_m_{45.0};
  int max_depth_delta_pwm_{100}, buoyancy_hold_delta_pwm_{40};
  double vision_throttle_weight_{0.4};
  int throttle_channel_{3}, yaw_channel_{4}, forward_channel_{5};
  int neutral_pwm_{1500}, min_pwm_{1300}, max_pwm_{1700};
  int max_yaw_delta_{180}, max_tracking_depth_delta_{100};
  bool yaw_invert_{false}, vertical_positive_is_up_{true};

  bool enabled_{false};
  bool control_granted_{false};
  State state_{State::IDLE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time operation_started_at_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> condition_started_at_;
  std::optional<Detection> buoy_, stick_;
  std::optional<double> depth_m_, hold_depth_m_;
  rclcpp::Time depth_received_at_{0, 0, RCL_ROS_TIME};
  double prestrike_buoy_area_{0.0};
  int target_retries_{0};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_granted_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_, rc_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_, failed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_vision_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<auv_vision_nav_control::BuoyVisualStrikeNode>();
  rclcpp::spin(node);
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
