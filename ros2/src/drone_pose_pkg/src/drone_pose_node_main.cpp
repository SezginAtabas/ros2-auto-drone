#include "drone_pose_node.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DronePoseNode>();

    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << std::string("::Exception::") << e.what();
  }
  return 0;
}