cmake_minimum_required(VERSION 3.8)
project(unitree_g1_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_launch test/test_launch_description.py)
endif()

ament_package()
