from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("kmu26_auv_planning_vision_control"),
                    "config",
                    "buoy_mission.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument("arena_length_m", default_value="10.0"),
        DeclareLaunchArgument("arena_width_m", default_value="15.0"),
        DeclareLaunchArgument("arena_depth_m", default_value="11.0"),
        DeclareLaunchArgument("arena_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("arena_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("arena_surface_z_m", default_value="0.0"),
        DeclareLaunchArgument(
            "arena_start_corner",
            default_value="bottom_left",
            description=(
                "Same convention as region_local_gradient_homing: "
                "bottom_left uses odom -Y inward, bottom_right uses +Y."
            ),
        ),
        DeclareLaunchArgument("arena_safety_margin_m", default_value="0.5"),
        DeclareLaunchArgument("surface_safety_margin_m", default_value="0.5"),
        DeclareLaunchArgument("bottom_safety_margin_m", default_value="1.0"),
        DeclareLaunchArgument("buoy_class_id", default_value="0"),
        DeclareLaunchArgument("stick_class_id", default_value="1"),
        DeclareLaunchArgument("standoff_distance_m", default_value="1.5"),
        DeclareLaunchArgument("mission_start_topic", default_value="/mission/start"),
        DeclareLaunchArgument("acoustic_handoff_enabled", default_value="true"),
        DeclareLaunchArgument(
            "vision_search_topic", default_value="/homing/vision_search_active"
        ),
        DeclareLaunchArgument(
            "target_confirmed_topic", default_value="/vision/target_confirmed"
        ),
        DeclareLaunchArgument(
            "control_granted_topic",
            default_value="/homing/vision_control_granted",
        ),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("request_vision_mode", default_value="true"),
        DeclareLaunchArgument("vision_mode_name", default_value="STABILIZE"),
        DeclareLaunchArgument("visual_dry_run", default_value="false"),
        DeclareLaunchArgument("depth_pose_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("throttle_channel", default_value="3"),
        DeclareLaunchArgument("yaw_channel", default_value="4"),
        DeclareLaunchArgument("forward_channel", default_value="5"),
        DeclareLaunchArgument("fork_target_x", default_value="0.30"),
        DeclareLaunchArgument("fork_target_y", default_value="0.70"),
        DeclareLaunchArgument("insert_pwm", default_value="1560"),
        DeclareLaunchArgument("strike_pwm", default_value="1620"),
        DeclareLaunchArgument("retract_pwm", default_value="1420"),
        DeclareLaunchArgument(
            "waypoint_topic",
            default_value="/waypoint",
            description=(
                "Odom-frame waypoint output for an external GUIDED controller."
            ),
        ),
    ]
    manager = Node(
        package="kmu26_auv_planning_vision_control",
        executable="buoy_mission_manager_node",
        name="buoy_mission_manager",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "arena_length_m": typed("arena_length_m", float),
                "arena_width_m": typed("arena_width_m", float),
                "arena_depth_m": typed("arena_depth_m", float),
                "arena_offset_x_m": typed("arena_offset_x_m", float),
                "arena_offset_y_m": typed("arena_offset_y_m", float),
                "arena_surface_z_m": typed("arena_surface_z_m", float),
                "arena_start_corner": LaunchConfiguration("arena_start_corner"),
                "arena_safety_margin_m": typed("arena_safety_margin_m", float),
                "surface_safety_margin_m": typed("surface_safety_margin_m", float),
                "bottom_safety_margin_m": typed("bottom_safety_margin_m", float),
                "buoy_class_id": typed("buoy_class_id", int),
                "stick_class_id": typed("stick_class_id", int),
                "standoff_distance_m": typed("standoff_distance_m", float),
                "mission_start_topic": LaunchConfiguration(
                    "mission_start_topic"
                ),
                "acoustic_handoff_enabled": typed(
                    "acoustic_handoff_enabled", bool
                ),
                "vision_search_topic": LaunchConfiguration(
                    "vision_search_topic"
                ),
                "target_confirmed_topic": LaunchConfiguration(
                    "target_confirmed_topic"
                ),
                "control_granted_topic": LaunchConfiguration(
                    "control_granted_topic"
                ),
                "dry_run": typed("dry_run", bool),
                "request_vision_mode": typed("request_vision_mode", bool),
                "vision_mode_name": LaunchConfiguration("vision_mode_name"),
                "waypoint_topic": LaunchConfiguration(
                    "waypoint_topic"
                ),
            }
        ],
    )
    visual_strike = Node(
        package="kmu26_auv_planning_vision_control",
        executable="buoy_visual_strike_node",
        name="buoy_visual_strike",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "dry_run": typed("visual_dry_run", bool),
                "acoustic_handoff_enabled": typed(
                    "acoustic_handoff_enabled", bool
                ),
                "control_granted_topic": LaunchConfiguration(
                    "control_granted_topic"
                ),
                "buoy_class_id": typed("buoy_class_id", int),
                "stick_class_id": typed("stick_class_id", int),
                "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                "throttle_channel": typed("throttle_channel", int),
                "yaw_channel": typed("yaw_channel", int),
                "forward_channel": typed("forward_channel", int),
                "fork_target_x": typed("fork_target_x", float),
                "fork_target_y": typed("fork_target_y", float),
                "insert_pwm": typed("insert_pwm", int),
                "strike_pwm": typed("strike_pwm", int),
                "retract_pwm": typed("retract_pwm", int),
            }
        ],
    )
    return LaunchDescription(arguments + [manager, visual_strike])
