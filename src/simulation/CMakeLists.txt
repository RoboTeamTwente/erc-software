cmake_minimum_required(VERSION 3.20)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(rtt_rover_control SHARED DIRECTORY src/control)

ament_auto_add_library(rtt_rover_driver SHARED DIRECTORY src/driver)
target_link_libraries(rtt_rover_driver rtt_rover_control)

install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(webots_ros2_driver plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
