from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
                description="Compressed camera topic received from the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/vision/buoy_detection_2d",
                description="YOLO 2D detection topic consumed by the depth range node.",
            ),
            DeclareLaunchArgument(
                "detection_3d_topic",
                default_value="/vision/buoy_detection_3d",
                description="Depth-enriched detections consumed by the coordinate mapper.",
            ),
            DeclareLaunchArgument(
                "annotated_image_topic",
                default_value="/vision/yolo/annotated/compressed",
                description="Single final RGB topic containing YOLO bbox and depth range overlay.",
            ),
            DeclareLaunchArgument("annotated_jpeg_quality", default_value="80"),
            DeclareLaunchArgument(
                "show_output_window",
                default_value="true",
                description="Show the final bbox and depth range overlay in an OpenCV window.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Required .pt model path. Example: /home/user/models/yolo26m_underwater_batch4_last.pt",
            ),
            DeclareLaunchArgument(
                "target_class_name",
                default_value="",
                description="Target class name. Leave empty to accept every detected class.",
            ),
            DeclareLaunchArgument(
                "target_class_id",
                default_value="-1",
                description="Target class id. Overrides target_class_name when >= 0.",
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument(
                "device",
                default_value="auto",
                description="Inference device: auto, cpu, cuda:0, etc.",
            ),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument(
                "show_preview",
                default_value="true",
                description="Show OpenCV preview window with live detections.",
            ),
            DeclareLaunchArgument(
                "preview_window_name",
                default_value="YOLO Buoy Detection",
                description="OpenCV window title for the preview UI.",
            ),
            DeclareLaunchArgument(
                "publish_per_class",
                default_value="true",
                description="Publish the best detection for every visible class in each frame.",
            ),
            DeclareLaunchArgument(
                "publish_all_targets",
                default_value="true",
                description="Publish every matching box, including multiple buoys of the same class.",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="RealSense depth aligned to the color image; requires align_depth.enable:=true.",
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/camera/camera/aligned_depth_to_color/camera_info",
                description="CameraInfo for the aligned depth image.",
            ),
            DeclareLaunchArgument("min_range_m", default_value="0.25"),
            DeclareLaunchArgument("max_range_m", default_value="6.0"),
            DeclareLaunchArgument(
                "depth_roi_scale",
                default_value="0.55",
                description="Central fraction of each YOLO box used for depth sampling.",
            ),
            DeclareLaunchArgument(
                "min_valid_depth_pixels",
                default_value="8",
                description="Minimum valid sampled depth points; kept low for sparse underwater depth.",
            ),
            DeclareLaunchArgument("max_detection_depth_sync_sec", default_value="0.25"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument(
                "publish_camera_mount_tf",
                default_value="true",
                description=(
                    "Publish the configurable parent-to-camera_link static TF. "
                    "Disable it when another launch already publishes this transform."
                ),
            ),
            DeclareLaunchArgument(
                "camera_mount_parent_frame",
                default_value="fcu_link",
                description=(
                    "Camera mount parent. fcu_link assumes the camera is mounted "
                    "at the Pixhawk pose recorded in the localization TF tree."
                ),
            ),
            DeclareLaunchArgument(
                "camera_mount_child_frame",
                default_value="camera_link",
            ),
            DeclareLaunchArgument("camera_mount_x_m", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_y_m", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_z_m", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_pitch_rad", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("arena_length_m", default_value="10.0"),
            DeclareLaunchArgument("arena_width_m", default_value="15.0"),
            DeclareLaunchArgument("arena_depth_m", default_value="11.0"),
            DeclareLaunchArgument(
                "arena_start_frame_topic",
                default_value="/guided/start_frame",
            ),
            DeclareLaunchArgument(
                "arena_start_corner",
                default_value="bottom_left",
                description=(
                    "bottom_left uses odom -Y inward; bottom_right uses +Y."
                ),
            ),
            DeclareLaunchArgument(
                "tracks_topic", default_value="/mission/buoy_tracks"
            ),
            DeclareLaunchArgument(
                "observation_topic", default_value="/mission/buoy_observation"
            ),
            DeclareLaunchArgument(
                "vision_search_topic",
                default_value="/homing/vision_search_active",
            ),
            DeclareLaunchArgument(
                "target_confirmed_topic",
                default_value="/vision/target_confirmed",
            ),
            DeclareLaunchArgument("buoy_class_id", default_value="0"),
            DeclareLaunchArgument("association_distance_m", default_value="1.0"),
            DeclareLaunchArgument("min_confirm_observations", default_value="6"),
            DeclareLaunchArgument("max_position_rms_m", default_value="0.30"),
            DeclareLaunchArgument("handoff_confirm_hits", default_value="4"),
            DeclareLaunchArgument(
                "depth_sample_stride",
                default_value="1",
                description="Pixel stride inside the YOLO ROI. 1 reads every aligned depth pixel.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_static_tf",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("publish_camera_mount_tf")
                ),
                arguments=[
                    "--x",
                    LaunchConfiguration("camera_mount_x_m"),
                    "--y",
                    LaunchConfiguration("camera_mount_y_m"),
                    "--z",
                    LaunchConfiguration("camera_mount_z_m"),
                    "--roll",
                    LaunchConfiguration("camera_mount_roll_rad"),
                    "--pitch",
                    LaunchConfiguration("camera_mount_pitch_rad"),
                    "--yaw",
                    LaunchConfiguration("camera_mount_yaw_rad"),
                    "--frame-id",
                    LaunchConfiguration("camera_mount_parent_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("camera_mount_child_frame"),
                ],
            ),
            Node(
                package="kmu26_auv_planning_vision_control",
                executable="yolo_buoy_detector",
                name="yolo_buoy_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class_name": LaunchConfiguration("target_class_name"),
                        "target_class_id": ParameterValue(LaunchConfiguration("target_class_id"), value_type=int),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                        "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
                        "preview_window_name": LaunchConfiguration("preview_window_name"),
                        "publish_per_class": ParameterValue(
                            LaunchConfiguration("publish_per_class"), value_type=bool
                        ),
                        "publish_all_targets": ParameterValue(
                            LaunchConfiguration("publish_all_targets"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="kmu26_auv_planning_vision_control",
                executable="depth_range_node",
                name="depth_range_node",
                output="screen",
                parameters=[
                    {
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "detection_3d_topic": LaunchConfiguration(
                            "detection_3d_topic"
                        ),
                        "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                        "depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic"),
                        "min_range_m": ParameterValue(LaunchConfiguration("min_range_m"), value_type=float),
                        "max_range_m": ParameterValue(LaunchConfiguration("max_range_m"), value_type=float),
                        "roi_scale": ParameterValue(LaunchConfiguration("depth_roi_scale"), value_type=float),
                        "min_valid_depth_pixels": ParameterValue(
                            LaunchConfiguration("min_valid_depth_pixels"), value_type=int
                        ),
                        "max_detection_depth_sync_sec": ParameterValue(
                            LaunchConfiguration("max_detection_depth_sync_sec"), value_type=float
                        ),
                        "sample_stride": ParameterValue(
                            LaunchConfiguration("depth_sample_stride"), value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="kmu26_auv_planning_vision_control",
                executable="yolo_range_overlay_node",
                name="yolo_range_overlay_node",
                output="screen",
                parameters=[
                    {
                        "rgb_image_topic": LaunchConfiguration("image_topic"),
                        "detection_3d_topic": LaunchConfiguration(
                            "detection_3d_topic"
                        ),
                        "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
                        "annotated_jpeg_quality": ParameterValue(
                            LaunchConfiguration("annotated_jpeg_quality"), value_type=int
                        ),
                        "show_window": ParameterValue(
                            LaunchConfiguration("show_output_window"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="kmu26_auv_planning_vision_control",
                executable="buoy_coordinate_mapper_node",
                name="buoy_coordinate_mapper",
                output="screen",
                parameters=[
                    {
                        "detection_3d_topic": LaunchConfiguration(
                            "detection_3d_topic"
                        ),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "arena_length_m": ParameterValue(
                            LaunchConfiguration("arena_length_m"), value_type=float
                        ),
                        "arena_width_m": ParameterValue(
                            LaunchConfiguration("arena_width_m"), value_type=float
                        ),
                        "arena_depth_m": ParameterValue(
                            LaunchConfiguration("arena_depth_m"), value_type=float
                        ),
                        "arena_start_frame_topic": LaunchConfiguration(
                            "arena_start_frame_topic"
                        ),
                        "arena_start_corner": LaunchConfiguration(
                            "arena_start_corner"
                        ),
                        "tracks_topic": LaunchConfiguration("tracks_topic"),
                        "observation_topic": LaunchConfiguration(
                            "observation_topic"
                        ),
                        "vision_search_topic": LaunchConfiguration(
                            "vision_search_topic"
                        ),
                        "target_confirmed_topic": LaunchConfiguration(
                            "target_confirmed_topic"
                        ),
                        "buoy_class_id": ParameterValue(
                            LaunchConfiguration("buoy_class_id"), value_type=int
                        ),
                        "association_distance_m": ParameterValue(
                            LaunchConfiguration("association_distance_m"),
                            value_type=float,
                        ),
                        "min_confirm_observations": ParameterValue(
                            LaunchConfiguration("min_confirm_observations"),
                            value_type=int,
                        ),
                        "max_position_rms_m": ParameterValue(
                            LaunchConfiguration("max_position_rms_m"),
                            value_type=float,
                        ),
                        "handoff_confirm_hits": ParameterValue(
                            LaunchConfiguration("handoff_confirm_hits"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
