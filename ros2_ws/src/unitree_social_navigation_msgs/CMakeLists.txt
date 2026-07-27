cmake_minimum_required(VERSION 3.8)
project(unitree_social_navigation_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictedTrajectories.msg"
  "msg/PredictedTrajectory.msg"
  "msg/PredictedTrajectoryPoint.msg"
  "msg/TrackedPerson.msg"
  "msg/TrackedPersons.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
