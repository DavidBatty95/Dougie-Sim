cmake_minimum_required(VERSION 3.8)
project(g1_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(g1_manipulation_geometry src/geometry.cpp)
target_include_directories(g1_manipulation_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(g1_manipulation_server_node src/manipulation_server.cpp)
target_include_directories(g1_manipulation_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(g1_manipulation_server_node g1_manipulation_geometry)
ament_target_dependencies(g1_manipulation_server_node
  g1_msgs geometry_msgs moveit_msgs moveit_ros_planning_interface rclcpp rclcpp_action
  shape_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros vision_msgs)

add_executable(g1_manipulation_server src/main.cpp)
target_link_libraries(g1_manipulation_server g1_manipulation_server_node)
ament_target_dependencies(g1_manipulation_server rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS
  g1_manipulation_geometry g1_manipulation_server_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS g1_manipulation_server DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry g1_manipulation_geometry)

  ament_add_gtest(test_detection test/test_detection.cpp)
  target_link_libraries(test_detection g1_manipulation_server_node)

  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  g1_msgs geometry_msgs moveit_msgs moveit_ros_planning_interface rclcpp rclcpp_action
  shape_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros vision_msgs)
ament_package()
