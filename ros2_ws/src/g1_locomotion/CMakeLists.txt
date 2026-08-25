cmake_minimum_required(VERSION 3.8)
project(g1_locomotion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(g1_base_approach
  src/g1_base_approach_node.cpp
  src/approach_planner.cpp
)
target_include_directories(g1_base_approach PRIVATE include)
ament_target_dependencies(g1_base_approach
  g1_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g1_base_approach PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS g1_base_approach DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md NOTICE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_approach_planner
    test/test_approach_planner.cpp
    src/approach_planner.cpp
  )
  target_include_directories(test_approach_planner PRIVATE include)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_approach_planner PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_package()
