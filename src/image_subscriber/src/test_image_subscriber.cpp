#include <vector>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include "../include/image_subscriber/test_image_subscriber.hpp"

TestImageSubscriber::TestImageSubscriber(
  const rclcpp::NodeOptions& options
)
: TestImageSubscriber("", options){}

TestImageSubscriber::TestImageSubscriber(
    const std::string& namespace_,
    const rclcpp::NodeOptions&)
    : rclcpp_lifecycle::LifecycleNode(
        "test_image_subs", 
        namespace_,
        rclcpp::NodeOptions().use_intra_process_comms(false)),
      param_node_(nullptr) {

  RCLCPP_INFO(get_logger(), "constructor");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "on_configure start");

  init();

  RCLCPP_INFO(this->get_logger(), "on_configure end");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate start");

  if (!camera_size()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "on_activate end");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown");

  close_window(tracking_window_name);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool TestImageSubscriber::capture_target(const sensor_msgs::msg::Image::SharedPtr& msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  roi_ = selectROI(tracking_window_name, image);
  cv::Mat target(image, roi_);
  if (target.empty()){
    return false;
  }
  cv::imwrite("/home/t-matsuo/target.jpeg", target);
  //RCLCPP_INFO(get_logger(), "(x, y, width, height) = (%lf, %lf, %lf, %lf)", roi.x, roi.y, roi.width, roi.height);

  // Trackerの初期化
  tracker_->init(image, roi_);

  // Trackerの色
  colorkcf = cv::Scalar(0, 255, 0);
  colorkcf2  = cv::Scalar(0, 255, 255);
      
  //cv::imshow("image", image);
  cv::waitKey(1);
  captured = true;
  targetSize = roi_.size();

  return true;
}

void TestImageSubscriber::tracking_target(const sensor_msgs::msg::Image::SharedPtr& msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    
  // 更新
  bool tracking_success = tracker_->update(image, roi_);

  // 位置判定の線(left)
  auto color_line = cv::Scalar(255, 0, 0);
  cv::Point2d left_top(left_line, 0);
  cv::Point2d left_bottom(left_line, camera_height);
  cv::line(image, left_top, left_bottom, color_line, 1, 1);

  // 位置判定の線(right)
  cv::Point2d right_top(right_line, 0);
  cv::Point2d right_bottom(right_line, camera_height);
  cv::line(image, right_top, right_bottom, color_line, 1, 1);

  // ターゲットの枠
  cv::rectangle(image, roi_, colorkcf, 1, 1);

  if (tracking_success) {
    cv::putText(image, "- KCF", cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, .5, colorkcf, 1, 16);
  } else {
    cv::putText(image, "Tracking Failed.", cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, .5, colorkcf, 1, 16);
  }
  cv::imshow(tracking_window_name, image);
  cv::waitKey(1);

  auto horizon = horizon_position(roi_);
  auto depth = depth_position(roi_);
  command(horizon, depth);
}

void TestImageSubscriber::command(
    TestImageSubscriber::HorizonPosition horizon, 
    TestImageSubscriber::DepthPosition depth) {

  switch (horizon) {
  case TestImageSubscriber::HorizonPosition::Left:
    RCLCPP_INFO(get_logger(), "turn right");
    break;
  
  case TestImageSubscriber::HorizonPosition::Right:
    RCLCPP_INFO(get_logger(), "turn left");
    break;
  
  case TestImageSubscriber::HorizonPosition::Center:
    RCLCPP_INFO(get_logger(), "stop");

    switch (depth) {
    case  TestImageSubscriber::DepthPosition::Far:
      RCLCPP_INFO(get_logger(), "farword");
      break;
    
    case  TestImageSubscriber::DepthPosition::Near:
      RCLCPP_INFO(get_logger(), "back");
      break;

    case  TestImageSubscriber::DepthPosition::Middle:
      RCLCPP_INFO(get_logger(), "stop");
      break;
    }
    break;
  }
}

void TestImageSubscriber::subscribe_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  switch (this->get_current_state().id())
  {
  case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
    if (!captured) {
      if (!capture_target(msg)) {
        this->deactivate();
      };
    } else {
      tracking_target(msg);
    }
    break;

  case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
    tracking_target(msg);
    break;

  default:
    break;
  }
}  

void TestImageSubscriber::init(void) {
  RCLCPP_INFO(get_logger(), "init");

  captured = false;
  tracker_ = cv::TrackerMedianFlow::create();
  subscription = create_subscription<sensor_msgs::msg::Image>(
      "image_raw",
      rclcpp::QoS(10),
      std::bind(&TestImageSubscriber::subscribe_image, this, std::placeholders::_1));
  
  clnt_ = this->create_client<rcl_interfaces::srv::GetParameters>("v4l2_camera/get_parameters");

  if (param_node_ == nullptr) {
    param_node_ = rclcpp::Node::make_shared("v4l2_camera_param");
    rclcpp::spin_some(param_node_);
  }

  ratio_[HorizonPosition::Left] = 0.25;
  ratio_[HorizonPosition::Center] = 0.50;
  ratio_[HorizonPosition::Right] = 0.25;
  declare_parameter("ratio", ratio_);

  ratio_param_handler_ = add_on_set_parameters_callback(
      std::bind(&TestImageSubscriber::reflect_param_changes, this, std::placeholders::_1)
  );
}

rcl_interfaces::msg::SetParametersResult
TestImageSubscriber::reflect_param_changes(const std::vector<rclcpp::Parameter>& params) {
  RCLCPP_INFO(get_logger(), "OK");
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = false;
  for (auto param : params) {
    if (param.get_name() == "ratio") {
      ratio_ = param.as_double_array();
      refresh_division_line();
      result.successful = true;
    }
  }
  return result;
}

cv::Point2d TestImageSubscriber::rect_center(cv::Rect2d rect) {
  return cv::Point2d(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
}

bool TestImageSubscriber::camera_size(void) {
  using namespace std::chrono_literals;

  auto param = rclcpp::SyncParametersClient(
      param_node_,
      "v4l2_camera");

  while(!param.wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service...");
  }

  auto param_res = param.get_parameters({"image_size"});
  camera_width = param_res[0].as_integer_array()[0];
  camera_height = param_res[0].as_integer_array()[1];
  RCLCPP_INFO(get_logger(),
      "camera size(width, height) = (%d, %d)", 
      this->camera_width, this->camera_height);

  refresh_division_line();

  return true;
}

void TestImageSubscriber::refresh_division_line() {
  left_line = camera_width * ratio_[0];
  right_line = camera_width * (ratio_[0] + ratio_[1]);
}

bool TestImageSubscriber::exist_window(std::string window_name) {
  return cv::getWindowProperty(window_name, cv::WINDOW_FULLSCREEN) >= 0.0;
}

void TestImageSubscriber::close_window(std::string window_name) {
  if (exist_window(window_name)) {
    cv::destroyWindow(window_name);
  }
}

TestImageSubscriber::HorizonPosition TestImageSubscriber::horizon_position(cv::Rect2d& target){
  auto target_center = rect_center(target);

  if (target_center.x < left_line) {
    return HorizonPosition::Left;
  } else if (target_center.x > right_line) {
    return HorizonPosition::Right;
  } else {
    return HorizonPosition::Center;
  }
}

TestImageSubscriber::DepthPosition TestImageSubscriber::depth_position(cv::Rect2d& /*target*/) {
  return TestImageSubscriber::DepthPosition::Middle;
}
