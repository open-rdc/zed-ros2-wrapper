#ifndef ZED_IMAGE_PUBLISHER__ZED_CAMERA_NODE_HPP_
#define ZED_IMAGE_PUBLISHER__ZED_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sl/Camera.hpp>

#include <memory>
#include <thread>

namespace zed_image_publisher
{

class ZedCameraNode : public rclcpp::Node
{
public:
  explicit ZedCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ZedCameraNode();

private:
  void initializeCamera();
  void shutdownCamera();
  void grabAndPublishFrame();
  void publishImage(const sl::Mat & zed_image, const rclcpp::Time & timestamp);
  void publishCameraInfo(const rclcpp::Time & timestamp);
  sensor_msgs::msg::CameraInfo createCameraInfoMsg();

  // ZED Camera
  std::unique_ptr<sl::Camera> zed_camera_;
  sl::InitParameters init_params_;
  sl::RuntimeParameters runtime_params_;
  sl::Mat zed_image_;

  // ROS2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Threading
  std::thread capture_thread_;
  std::atomic<bool> capture_running_;

  // Parameters
  std::string frame_id_;
  int camera_id_;
  bool use_gpu_;
  double fps_;
  int image_width_;
  int image_height_;
  int video_quality_;
  double brightness_;
  double contrast_;
  double hue_;
  double saturation_;
  double sharpness_;
  double gamma_;
  bool auto_exposure_;
  int exposure_;
  bool auto_whitebalance_;
  int whitebalance_temperature_;

  // Camera info
  sensor_msgs::msg::CameraInfo camera_info_msg_;
};

}  // namespace zed_image_publisher

#endif  // ZED_IMAGE_PUBLISHER__ZED_CAMERA_NODE_HPP_