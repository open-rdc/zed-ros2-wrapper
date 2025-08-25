#include "zed_image_publisher/zed_camera_node.hpp"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>

namespace zed_image_publisher
{

ZedCameraNode::ZedCameraNode(const rclcpp::NodeOptions & options)
: Node("zed_camera_node", options),
  capture_running_(false)
{
  RCLCPP_INFO(get_logger(), "Initializing ZED Camera Simple Node");

  // Declare parameters
  declare_parameter("frame_id", "zed_left_camera_frame");
  declare_parameter("camera_id", 0);
  declare_parameter("use_gpu", true);
  declare_parameter("fps", 30.0);
  declare_parameter("image_width", 1280);
  declare_parameter("image_height", 720);
  declare_parameter("video_quality", 2);  // 0=HIGH, 1=MEDIUM, 2=LOW
  declare_parameter("brightness", 4.0);
  declare_parameter("contrast", 4.0);
  declare_parameter("hue", 0.0);
  declare_parameter("saturation", 4.0);
  declare_parameter("sharpness", 4.0);
  declare_parameter("gamma", 8.0);
  declare_parameter("auto_exposure", true);
  declare_parameter("exposure", 100);
  declare_parameter("auto_whitebalance", true);
  declare_parameter("whitebalance_temperature", 4200);

  // Get parameters
  get_parameter("frame_id", frame_id_);
  get_parameter("camera_id", camera_id_);
  get_parameter("use_gpu", use_gpu_);
  get_parameter("fps", fps_);
  get_parameter("image_width", image_width_);
  get_parameter("image_height", image_height_);
  get_parameter("video_quality", video_quality_);
  get_parameter("brightness", brightness_);
  get_parameter("contrast", contrast_);
  get_parameter("hue", hue_);
  get_parameter("saturation", saturation_);
  get_parameter("sharpness", sharpness_);
  get_parameter("gamma", gamma_);
  get_parameter("auto_exposure", auto_exposure_);
  get_parameter("exposure", exposure_);
  get_parameter("auto_whitebalance", auto_whitebalance_);
  get_parameter("whitebalance_temperature", whitebalance_temperature_);

  // Initialize publishers  
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("zed/rgb/image_raw", 10);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("zed/rgb/camera_info", 10);

  // Initialize ZED camera
  initializeCamera();
}

ZedCameraNode::~ZedCameraNode()
{
  shutdownCamera();
}

void ZedCameraNode::initializeCamera()
{
  RCLCPP_INFO(get_logger(), "Initializing ZED Camera...");

  zed_camera_ = std::make_unique<sl::Camera>();

  // Set initialization parameters
  init_params_.camera_resolution = sl::RESOLUTION::HD720;
  if (image_width_ == 1920 && image_height_ == 1080) {
    init_params_.camera_resolution = sl::RESOLUTION::HD1080;
  } else if (image_width_ == 1280 && image_height_ == 720) {
    init_params_.camera_resolution = sl::RESOLUTION::HD720;
  } else if (image_width_ == 672 && image_height_ == 376) {
    init_params_.camera_resolution = sl::RESOLUTION::VGA;
  }

  init_params_.camera_fps = static_cast<int>(fps_);
  init_params_.input.setFromCameraID(camera_id_);
  
  // Set depth mode to NONE for RGB-only operation
  init_params_.depth_mode = sl::DEPTH_MODE::NONE;
  
  // Set coordinate system
  init_params_.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
  init_params_.coordinate_units = sl::UNIT::METER;

  // GPU/CPU usage
  if (!use_gpu_) {
    init_params_.sdk_gpu_id = -1;  // Use CPU only
  }

  // Open camera
  sl::ERROR_CODE err = zed_camera_->open(init_params_);
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to open ZED camera: %s", 
                 sl::toString(err).c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "ZED Camera opened successfully");

  // Set camera settings
  if (video_quality_ >= 0 && video_quality_ <= 4) {
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, static_cast<int>(brightness_));
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, static_cast<int>(contrast_));
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::HUE, static_cast<int>(hue_));
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, static_cast<int>(saturation_));
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, static_cast<int>(sharpness_));
    zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, static_cast<int>(gamma_));
    
    if (auto_exposure_) {
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
    } else {
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, exposure_);
    }
    
    if (auto_whitebalance_) {
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 1);
    } else {
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
      zed_camera_->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, whitebalance_temperature_);
    }
  }

  // Create camera info message
  camera_info_msg_ = createCameraInfoMsg();

  // Start capture thread
  capture_running_ = true;
  capture_thread_ = std::thread(&ZedCameraNode::grabAndPublishFrame, this);

  RCLCPP_INFO(get_logger(), "ZED Camera initialization complete");
}

void ZedCameraNode::shutdownCamera()
{
  if (capture_running_) {
    capture_running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

  if (zed_camera_ && zed_camera_->isOpened()) {
    zed_camera_->close();
    RCLCPP_INFO(get_logger(), "ZED Camera closed");
  }
}

void ZedCameraNode::grabAndPublishFrame()
{
  RCLCPP_INFO(get_logger(), "Starting frame capture loop");

  while (rclcpp::ok() && capture_running_) {
    sl::ERROR_CODE err = zed_camera_->grab(runtime_params_);
    if (err == sl::ERROR_CODE::SUCCESS) {
      auto timestamp = now();
      
      // Retrieve left RGB image
      zed_camera_->retrieveImage(zed_image_, sl::VIEW::LEFT, sl::MEM::CPU);
      
      // Publish image and camera info
      publishImage(zed_image_, timestamp);
      publishCameraInfo(timestamp);
      
    } else {
      // Handle non-success cases (including no new frame)
      if (err != sl::ERROR_CODE::CAMERA_REBOOTING && err != sl::ERROR_CODE::CORRUPTED_FRAME) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Failed to grab frame: %s", sl::toString(err).c_str());
      }
    }
    
    // Sleep to maintain frame rate
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)));
  }

  RCLCPP_INFO(get_logger(), "Frame capture loop ended");
}

void ZedCameraNode::publishImage(const sl::Mat & zed_image, const rclcpp::Time & timestamp)
{
  if (image_pub_->get_subscription_count() > 0) {
    // Convert ZED Mat to OpenCV Mat
    cv::Mat cv_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>());
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGRA2RGB);
    
    // Convert to ROS message
    std_msgs::msg::Header header;
    header.stamp = timestamp;
    header.frame_id = frame_id_;
    
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "rgb8", rgb_image).toImageMsg();
    image_pub_->publish(*msg);
  }
}

void ZedCameraNode::publishCameraInfo(const rclcpp::Time & timestamp)
{
  if (camera_info_pub_->get_subscription_count() > 0) {
    camera_info_msg_.header.stamp = timestamp;
    camera_info_msg_.header.frame_id = frame_id_;
    camera_info_pub_->publish(camera_info_msg_);
  }
}

sensor_msgs::msg::CameraInfo ZedCameraNode::createCameraInfoMsg()
{
  sensor_msgs::msg::CameraInfo camera_info;
  
  // Get camera information from ZED
  sl::CalibrationParameters calib_params = zed_camera_->getCameraInformation().camera_configuration.calibration_parameters;
  
  camera_info.height = image_height_;
  camera_info.width = image_width_;
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  
  // Intrinsic camera matrix (K)
  camera_info.k[0] = calib_params.left_cam.fx;
  camera_info.k[2] = calib_params.left_cam.cx;
  camera_info.k[4] = calib_params.left_cam.fy;
  camera_info.k[5] = calib_params.left_cam.cy;
  camera_info.k[8] = 1.0;
  
  // Distortion coefficients (D)
  camera_info.d.resize(5);
  camera_info.d[0] = calib_params.left_cam.disto[0];  // k1
  camera_info.d[1] = calib_params.left_cam.disto[1];  // k2
  camera_info.d[2] = calib_params.left_cam.disto[4];  // p1
  camera_info.d[3] = calib_params.left_cam.disto[5];  // p2
  camera_info.d[4] = calib_params.left_cam.disto[2];  // k3
  
  // Rectification matrix (R) - Identity for left camera
  camera_info.r[0] = 1.0;
  camera_info.r[4] = 1.0;
  camera_info.r[8] = 1.0;
  
  // Projection matrix (P)
  camera_info.p[0] = calib_params.left_cam.fx;
  camera_info.p[2] = calib_params.left_cam.cx;
  camera_info.p[5] = calib_params.left_cam.fy;
  camera_info.p[6] = calib_params.left_cam.cy;
  camera_info.p[10] = 1.0;
  
  return camera_info;
}

}  // namespace zed_image_publisher