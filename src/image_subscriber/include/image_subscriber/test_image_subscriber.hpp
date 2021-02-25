#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/tracking.hpp>

class TestImageSubscriber : public rclcpp_lifecycle::LifecycleNode {
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    bool captured;
    cv::Rect2d prevRoi;
    cv::Size2d targetSize;
    cv::Ptr<cv::Tracker> trackerKCF;
    cv::Scalar colorkcf;
    cv::Scalar colorkcf2;
    void init(void);    
    void subscribe_image(const sensor_msgs::msg::Image::SharedPtr image);
    cv::Point2d rect_center(cv::Rect2d);

  public:
    TestImageSubscriber(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    TestImageSubscriber(
      const std::string& namespace_,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);
};