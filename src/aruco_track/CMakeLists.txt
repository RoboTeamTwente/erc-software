cmake_minimum_required(VERSION 3.20)
project(aruco_track)

# Compiler warnings (good practice)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Executable
add_executable(aruco_tf2_node
  src/aruco_tf2_node.cpp
)

# Include directories
target_include_directories(aruco_tf2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies (modern ROS2 way)
target_link_libraries(aruco_tf2_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${OpenCV_LIBRARIES}
)

install(
  FILES config/charuco_camera_params.yml
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(aruco_tf2_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBRARIES}
)


# Install
install(TARGETS aruco_tf2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
