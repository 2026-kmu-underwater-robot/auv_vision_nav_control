cmake_minimum_required(VERSION 3.8)
project(auv_vision_nav_control)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BuoyDetection2D.msg"
  "msg/BuoyDetection3D.msg"
  "msg/BuoyTrack.msg"
  "msg/BuoyTrackArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

add_executable(depth_range_node src/depth_range_node.cpp)
target_compile_features(depth_range_node PUBLIC cxx_std_17)
target_include_directories(depth_range_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_range_node ${OpenCV_LIBS})
ament_target_dependencies(depth_range_node
  cv_bridge geometry_msgs rclcpp sensor_msgs)
rosidl_target_interfaces(depth_range_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(yolo_range_overlay_node src/yolo_range_overlay_node.cpp)
target_compile_features(yolo_range_overlay_node PUBLIC cxx_std_17)
target_include_directories(yolo_range_overlay_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolo_range_overlay_node ${OpenCV_LIBS})
ament_target_dependencies(yolo_range_overlay_node cv_bridge rclcpp sensor_msgs)
rosidl_target_interfaces(yolo_range_overlay_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(buoy_coordinate_mapper_node src/buoy_coordinate_mapper_node.cpp)
target_compile_features(buoy_coordinate_mapper_node PUBLIC cxx_std_17)
ament_target_dependencies(buoy_coordinate_mapper_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)
rosidl_target_interfaces(
  buoy_coordinate_mapper_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(buoy_mission_manager_node src/buoy_mission_manager_node.cpp)
target_compile_features(buoy_mission_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(buoy_mission_manager_node
  geometry_msgs mavros_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)
rosidl_target_interfaces(buoy_mission_manager_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(buoy_visual_strike_node src/buoy_visual_strike_node.cpp)
target_compile_features(buoy_visual_strike_node PUBLIC cxx_std_17)
ament_target_dependencies(buoy_visual_strike_node
  geometry_msgs mavros_msgs rclcpp std_msgs)

add_executable(buoy_mission_visualization_node src/buoy_mission_visualization_node.cpp)
target_compile_features(buoy_mission_visualization_node PUBLIC cxx_std_17)
ament_target_dependencies(buoy_mission_visualization_node
  geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)
rosidl_target_interfaces(
  buoy_mission_visualization_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(PROGRAMS
  scripts/yolo_buoy_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_buoy_detector
)

install(TARGETS
  depth_range_node
  yolo_range_overlay_node
  buoy_coordinate_mapper_node
  buoy_mission_manager_node
  buoy_visual_strike_node
  buoy_mission_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_yolo_env.py
  scripts/setup_laptop_yolo_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
