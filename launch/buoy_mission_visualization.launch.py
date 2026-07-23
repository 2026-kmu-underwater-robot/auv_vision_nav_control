from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    package_share = FindPackageShare("kmu26_auv_planning_vision_control")
    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [package_share, "config", "buoy_mission.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [package_share, "rviz", "buoy_mission.rviz"]
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("arena_length_m", default_value="10.0"),
        DeclareLaunchArgument("arena_width_m", default_value="15.0"),
        DeclareLaunchArgument("arena_depth_m", default_value="11.0"),
        DeclareLaunchArgument("arena_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("arena_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("arena_surface_z_m", default_value="0.0"),
        DeclareLaunchArgument("arena_start_corner", default_value="bottom_left"),
        DeclareLaunchArgument("arena_frame", default_value="odom"),
    ]

    visualization = Node(
        package="kmu26_auv_planning_vision_control",
        executable="buoy_mission_visualization_node",
        name="buoy_mission_visualization",
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
                "arena_frame": LaunchConfiguration("arena_frame"),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="buoy_mission_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(arguments + [visualization, rviz])
