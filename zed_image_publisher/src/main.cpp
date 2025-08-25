#include <rclcpp/rclcpp.hpp>
#include "zed_image_publisher/zed_camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<zed_image_publisher::ZedCameraNode>();
  
  RCLCPP_INFO(node->get_logger(), "ZED Image Publisher node started");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}