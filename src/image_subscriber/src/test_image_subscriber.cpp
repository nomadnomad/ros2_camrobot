#include <cv_bridge/cv_bridge.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/qos.hpp>
#include "../include/image_subscriber/test_image_subscriber.hpp"

TestImageSubscriber::TestImageSubscriber(
  const rclcpp::NodeOptions& options
)
: TestImageSubscriber("", options){}

TestImageSubscriber::TestImageSubscriber(
   const std::string& namespace_,
   const rclcpp::NodeOptions&
)
: rclcpp_lifecycle::LifecycleNode("test_image_subs", 
      namespace_,
      rclcpp::NodeOptions().use_intra_process_comms(false)) {

    RCLCPP_INFO(get_logger(), "constructor");
}

void TestImageSubscriber::init(void) {
  RCLCPP_INFO(get_logger(), "init");
  captured = false;
  trackerKCF = cv::TrackerKCF::create();
  subscription = create_subscription<sensor_msgs::msg::Image>(
    "image_raw",
    rclcpp::QoS(10),
    std::bind(&TestImageSubscriber::subscribe_image, this, std::placeholders::_1)
  );
}

void TestImageSubscriber::subscribe_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }
    /*
    RCLCPP_INFO(get_logger(), "current_state[%d][%s] subscribe_image frame_id[%s] encoding[%s]",
      this->get_current_state().id(),
      this->get_current_state().label().c_str(), 
      msg->header.frame_id.c_str(),
      msg->encoding.c_str());
    */
    
    if (!captured) {
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Rect2d roi = selectROI("tracker", image);
      cv::Mat target(image, roi);
      cv::imwrite("/home/t-matsuo/target.jpeg", target);
      RCLCPP_INFO(get_logger(), "(x, y, width, height) = (%lf, %lf, %lf, %lf)", roi.x, roi.y, roi.width, roi.height);

      // Trackerの初期化
      trackerKCF->init(image, roi);

      // Trackerの色
      colorkcf = cv::Scalar(0, 255, 0);
      colorkcf2  = cv::Scalar(0, 255, 255);
      
      //cv::imshow("image", image);
      cv::waitKey(1);
      captured = true;
      prevRoi = roi;
      targetSize = roi.size();
    } else {
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      
      // 更新
      cv::Rect2d roi;
      trackerKCF->update(image, roi);
      RCLCPP_INFO(get_logger(), "(x, y, width, height) = (%lf, %lf, %lf, %lf)", roi.x, roi.y, roi.width, roi.height);

      // 矩形で囲む
      cv::rectangle(image, roi, colorkcf, 1, 1);

      cv::putText(image, "- KCF", cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, .5, colorkcf, 1, 16);

      cv::imshow("tracker", image);

      //cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      //cv::imshow("image", image);
      cv::waitKey(1);
      prevRoi = roi;
    }
    
}

cv::Point2d TestImageSubscriber::rect_center(cv::Rect2d rect) {
  return cv::Point2d(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "on_configure");

    init();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TestImageSubscriber::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "on_activate");

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

    cv::destroyWindow("image");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}