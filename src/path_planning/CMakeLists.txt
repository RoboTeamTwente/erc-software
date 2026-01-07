cmake_minimum_required(VERSION 3.20)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(path_planning DIRECTORY src)

install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  find_package(Catch2 3 REQUIRED)

  ament_auto_add_executable(path_planning_test DIRECTORY tests)
  target_link_libraries(path_planning_test Catch2::Catch2WithMain)

  include(CTest)
  include(Catch)
  catch_discover_tests(path_planning_test)
endif()

ament_auto_package()
