
#include <memory>
#include "my_components/server_component.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto attach_shelf_service_server = std::make_shared<my_components::MidLegsTFService>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(attach_shelf_service_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}