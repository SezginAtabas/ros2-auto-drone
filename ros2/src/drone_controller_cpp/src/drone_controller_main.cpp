
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "drone_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DroneControllerNode>();
    node->change_mode("GUIDED");
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    std::cerr << std::string("::Exception::") << e.what();
  }
  return 0;
}