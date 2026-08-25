cmake_minimum_required(VERSION 3.8)
project(g1_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApproachObject.action"
  "action/Pick.action"
  "action/Place.action"
  "action/Retreat.action"
  "action/SetArmPosture.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

install(FILES README.md NOTICE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
