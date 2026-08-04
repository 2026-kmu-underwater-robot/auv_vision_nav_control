#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "auv_vision_nav_control/msg/buoy_track.hpp"
#include "auv_vision_nav_control/msg/buoy_track_array.hpp"

namespace auv_vision_nav_control
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Track = msg::BuoyTrack;
using TrackArray = msg::BuoyTrackArray;

class BuoyMissionVisualization : public rclcpp::Node
{
public:
  BuoyMissionVisualization()
  : Node("buoy_mission_visualization")
  {
    search_length_ = declare_parameter<double>("arena_length_m", 10.0);
    search_width_ = declare_parameter<double>("arena_width_m", 15.0);
    search_depth_ = declare_parameter<double>("arena_depth_m", 11.0);
    arena_start_corner_ = declare_parameter<std::string>(
      "arena_start_corner", "bottom_left");
    arena_frame_ = declare_parameter<std::string>("arena_frame", "arena");
    max_observation_points_ = declare_parameter<int>("max_observation_points", 5000);
    max_path_points_ = declare_parameter<int>("max_path_points", 10000);
    path_min_step_m_ = declare_parameter<double>("path_min_step_m", 0.05);

    const auto odom_topic =
      declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    const auto tracks_topic =
      declare_parameter<std::string>("tracks_topic", "/mission/buoy_tracks");
    const auto observation_topic =
      declare_parameter<std::string>("observation_topic", "/mission/buoy_observation");
    const auto waypoint_topic =
      declare_parameter<std::string>("active_waypoint_topic", "/mission/active_waypoint");
    const auto state_topic =
      declare_parameter<std::string>("mission_state_topic", "/mission/state");
    const auto markers_topic =
      declare_parameter<std::string>("markers_topic", "/mission/visualization_markers");
    const auto actual_path_topic =
      declare_parameter<std::string>("actual_path_topic", "/mission/actual_path");

    if (
      search_length_ <= 0.0 || search_width_ <= 0.0 || search_depth_ <= 0.0 ||
      arena_frame_.empty() ||
      max_observation_points_ < 1 || max_path_points_ < 2 || path_min_step_m_ < 0.0)
    {
      throw std::invalid_argument("visualization parameters are invalid");
    }
    if (
      arena_start_corner_ != "bottom_left" &&
      arena_start_corner_ != "bottom_right")
    {
      throw std::invalid_argument(
              "arena_start_corner must be bottom_left or bottom_right");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&BuoyMissionVisualization::on_odom, this, std::placeholders::_1));
    tracks_sub_ = create_subscription<TrackArray>(
      tracks_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](TrackArray::SharedPtr message) {latest_tracks_ = *message;});
    observation_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      observation_topic, 50,
      std::bind(&BuoyMissionVisualization::on_observation, this, std::placeholders::_1));
    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      waypoint_topic, 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr message) {
        active_waypoint_ = *message;
      });
    state_sub_ = create_subscription<std_msgs::msg::String>(
      state_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) {mission_state_ = message->data;});

    markers_pub_ = create_publisher<MarkerArray>(markers_topic, 10);
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      actual_path_topic, rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&BuoyMissionVisualization::publish_visualization, this));

    RCLCPP_INFO(
      get_logger(),
      "Mission visualization ready: arena-local x=[0.00, %.2f] "
      "width=%.2f z=[%.2f, 0.00], frame=%s",
      search_length_, search_width_, -search_depth_,
      arena_frame_.c_str());
  }

private:
  static Marker marker_base(
    const std::string & frame, const std::string & marker_namespace,
    int32_t id, int32_t type, const rclcpp::Time & stamp)
  {
    Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  static void set_color(Marker & marker, float red, float green, float blue, float alpha)
  {
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
  }

  static double point_distance(
    const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odom_ = *message;
    if (message->header.frame_id.empty()) {
      return;
    }
    if (
      !actual_path_.poses.empty() &&
      point_distance(
        actual_path_.poses.back().pose.position,
        message->pose.pose.position) < path_min_step_m_)
    {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;
    actual_path_.header = message->header;
    actual_path_.poses.push_back(pose);
    if (static_cast<int>(actual_path_.poses.size()) > max_path_points_) {
      actual_path_.poses.erase(actual_path_.poses.begin());
    }
  }

  void on_observation(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      return;
    }
    if (!observation_frame_.empty() && observation_frame_ != message->header.frame_id) {
      observations_.clear();
    }
    observation_frame_ = message->header.frame_id;
    observations_.push_back(message->point);
    while (static_cast<int>(observations_.size()) > max_observation_points_) {
      observations_.pop_front();
    }
  }

  void add_volume_markers(MarkerArray & output, const rclcpp::Time & stamp) const
  {
    const double y_min =
      arena_start_corner_ == "bottom_left" ? -search_width_ : 0.0;
    const double y_max =
      arena_start_corner_ == "bottom_left" ? 0.0 : search_width_;
    const double z_min = -search_depth_;
    const double z_max = 0.0;
    auto volume = marker_base(arena_frame_, "search_volume", 0, Marker::CUBE, stamp);
    volume.pose.position =
      make_point(0.5 * search_length_, 0.5 * (y_min + y_max),
      0.5 * (z_min + z_max));
    volume.pose.orientation.w = 1.0;
    volume.scale.x = search_length_;
    volume.scale.y = search_width_;
    volume.scale.z = search_depth_;
    set_color(volume, 0.10F, 0.55F, 1.0F, 0.07F);
    output.markers.push_back(volume);

    auto edges = marker_base(
      arena_frame_, "search_volume", 1, Marker::LINE_LIST, stamp);
    edges.scale.x = 0.04;
    set_color(edges, 0.15F, 0.70F, 1.0F, 0.85F);
    const std::vector<geometry_msgs::msg::Point> corners = {
      make_point(0.0, y_min, z_max),
      make_point(search_length_, y_min, z_max),
      make_point(search_length_, y_max, z_max),
      make_point(0.0, y_max, z_max),
      make_point(0.0, y_min, z_min),
      make_point(search_length_, y_min, z_min),
      make_point(search_length_, y_max, z_min),
      make_point(0.0, y_max, z_min)};
    constexpr int pairs[][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto & pair : pairs) {
      edges.points.push_back(corners[static_cast<size_t>(pair[0])]);
      edges.points.push_back(corners[static_cast<size_t>(pair[1])]);
    }
    output.markers.push_back(edges);
  }

  static geometry_msgs::msg::Point make_point(double x, double y, double z)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  void add_observation_marker(MarkerArray & output, const rclcpp::Time & stamp) const
  {
    const std::string frame = observation_frame_.empty() ? "odom" : observation_frame_;
    auto marker = marker_base(frame, "raw_observations", 0, Marker::POINTS, stamp);
    marker.scale.x = 0.07;
    marker.scale.y = 0.07;
    set_color(marker, 1.0F, 0.55F, 0.0F, 0.45F);
    marker.points.assign(observations_.begin(), observations_.end());
    output.markers.push_back(marker);
  }

  static std::string track_status_name(uint8_t status)
  {
    switch (status) {
      case Track::CANDIDATE: return "CANDIDATE";
      case Track::CONFIRMED: return "CONFIRMED";
      case Track::ASSIGNED: return "ASSIGNED";
      case Track::APPROACHING: return "APPROACHING";
      case Track::SERVICED: return "SERVICED";
      case Track::STALE: return "STALE";
      case Track::LOST: return "LOST";
      case Track::REJECTED: return "REJECTED";
      default: return "UNKNOWN";
    }
  }

  static void set_track_color(Marker & marker, uint8_t status)
  {
    switch (status) {
      case Track::CANDIDATE:
        set_color(marker, 1.0F, 0.85F, 0.0F, 0.95F);
        break;
      case Track::CONFIRMED:
        set_color(marker, 1.0F, 0.10F, 0.10F, 1.0F);
        break;
      case Track::ASSIGNED:
      case Track::APPROACHING:
        set_color(marker, 0.10F, 0.35F, 1.0F, 1.0F);
        break;
      case Track::SERVICED:
        set_color(marker, 0.10F, 1.0F, 0.25F, 1.0F);
        break;
      case Track::LOST:
        set_color(marker, 1.0F, 0.0F, 0.85F, 0.75F);
        break;
      case Track::STALE:
      case Track::REJECTED:
      default:
        set_color(marker, 0.45F, 0.45F, 0.45F, 0.65F);
        break;
    }
  }

  void add_track_markers(MarkerArray & output, const rclcpp::Time & stamp) const
  {
    if (!latest_tracks_) {
      return;
    }
    const std::string frame = latest_tracks_->header.frame_id.empty() ?
      "odom" : latest_tracks_->header.frame_id;
    for (const auto & track : latest_tracks_->tracks) {
      auto sphere = marker_base(
        frame, "buoy_positions", static_cast<int32_t>(track.id), Marker::SPHERE, stamp);
      sphere.pose.position = track.position_mission;
      const double diameter = track.status == Track::CANDIDATE ? 0.25 : 0.45;
      sphere.scale.x = diameter;
      sphere.scale.y = diameter;
      sphere.scale.z = diameter;
      set_track_color(sphere, track.status);
      output.markers.push_back(sphere);

      auto text = marker_base(
        frame, "buoy_labels", static_cast<int32_t>(track.id), Marker::TEXT_VIEW_FACING, stamp);
      text.pose.position = track.position_mission;
      text.pose.position.z += 0.40;
      text.scale.z = 0.25;
      text.text =
        "B" + std::to_string(track.id) + " " + track_status_name(track.status) +
        " n=" + std::to_string(track.observation_count) +
        " rms=" + std::to_string(track.position_rms_m).substr(0, 4);
      set_color(text, 1.0F, 1.0F, 1.0F, 1.0F);
      output.markers.push_back(text);
    }
  }

  void add_robot_marker(MarkerArray & output, const rclcpp::Time & stamp) const
  {
    if (!latest_odom_ || latest_odom_->header.frame_id.empty()) {
      return;
    }
    auto robot = marker_base(
      latest_odom_->header.frame_id, "robot", 0, Marker::ARROW, stamp);
    robot.pose = latest_odom_->pose.pose;
    robot.scale.x = 0.80;
    robot.scale.y = 0.25;
    robot.scale.z = 0.25;
    set_color(robot, 1.0F, 0.55F, 0.05F, 1.0F);
    output.markers.push_back(robot);

    auto state = marker_base(
      latest_odom_->header.frame_id, "mission_state", 0, Marker::TEXT_VIEW_FACING, stamp);
    state.pose.position = latest_odom_->pose.pose.position;
    state.pose.position.z += 0.8;
    state.scale.z = 0.32;
    state.text = mission_state_;
    set_color(state, 1.0F, 1.0F, 1.0F, 1.0F);
    output.markers.push_back(state);
  }

  void add_waypoint_marker(MarkerArray & output, const rclcpp::Time & stamp) const
  {
    if (!active_waypoint_ || active_waypoint_->header.frame_id.empty()) {
      return;
    }
    auto waypoint = marker_base(
      active_waypoint_->header.frame_id, "active_waypoint", 0, Marker::SPHERE, stamp);
    waypoint.pose.position = active_waypoint_->point;
    waypoint.scale.x = 0.35;
    waypoint.scale.y = 0.35;
    waypoint.scale.z = 0.35;
    set_color(waypoint, 0.0F, 1.0F, 1.0F, 1.0F);
    output.markers.push_back(waypoint);
  }

  void publish_visualization()
  {
    const auto stamp = now();
    MarkerArray markers;
    add_volume_markers(markers, stamp);
    add_observation_marker(markers, stamp);
    add_track_markers(markers, stamp);
    add_robot_marker(markers, stamp);
    add_waypoint_marker(markers, stamp);
    markers_pub_->publish(markers);

    if (!actual_path_.header.frame_id.empty()) {
      actual_path_.header.stamp = stamp;
      actual_path_pub_->publish(actual_path_);
    }
  }

  double search_length_{10.0};
  double search_width_{15.0};
  double search_depth_{11.0};
  int max_observation_points_{5000};
  int max_path_points_{10000};
  double path_min_step_m_{0.05};
  std::string mission_state_{"IDLE"};
  std::string arena_start_corner_{"bottom_left"};
  std::string arena_frame_{"arena"};
  std::string observation_frame_;
  std::deque<geometry_msgs::msg::Point> observations_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<TrackArray> latest_tracks_;
  std::optional<geometry_msgs::msg::PointStamped> active_waypoint_;
  nav_msgs::msg::Path actual_path_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr observation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_vision_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      auv_vision_nav_control::BuoyMissionVisualization>());
  rclcpp::shutdown();
  return 0;
}
