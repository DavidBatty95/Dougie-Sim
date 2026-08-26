#include "g1_manipulation/manipulation_server.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_manipulation::ManipulationServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int status = 0;
  try {
    node->initialize();
    spinner.join();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "manipulation initialization failed: %s", error.what());
    status = 1;
    rclcpp::shutdown();
    spinner.join();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return status;
}
