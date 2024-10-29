
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "drone_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DroneControllerNode>();
    std::uint32_t message_id = 32;
    float message_rate = 100000;
    node->request_message_interval(message_id, message_rate);
    rclcpp::spin(node);

  } catch (const std::exception &e) {
    std::cerr << std::string("::Exception::") << e.what();
  }
  return 0;
}