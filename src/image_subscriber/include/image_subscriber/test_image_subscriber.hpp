#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/tracking.hpp>

class TestImageSubscriber : public rclcpp_lifecycle::LifecycleNode {
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

    enum HorizonPosition { Left, Center, Right };
    enum DepthPosition { Far, Middle, Near };

  private:
    const std::string tracking_window_name = "tracking";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr clnt_;
    bool captured;
    cv::Rect2d prevRoi;
    cv::Size2d targetSize;
    //cv::Ptr<cv::Tracker> trackerKCF;
    cv::Ptr<cv::Tracker> tracker_;
    cv::Scalar colorkcf;
    cv::Scalar colorkcf2;
    int camera_height;
    rclcpp::Node::SharedPtr param_node_;
    std::vector<double> ratio_ = std::vector<double>(3, 0.0);
    int left_line;
    int right_line;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ratio_param_handler_;
    cv::Rect2d roi_;

    int camera_width;
    void init(void);    
    void subscribe_image(const sensor_msgs::msg::Image::SharedPtr image);
    cv::Point2d rect_center(cv::Rect2d);
    bool capture_target(const sensor_msgs::msg::Image::SharedPtr& msg);
    void tracking_target(const sensor_msgs::msg::Image::SharedPtr& msg);
    bool camera_size(void);
    bool exist_window(std::string window_name);
    void close_window(std::string window_name);
    HorizonPosition horizon_position(cv::Rect2d& /*target*/);
    DepthPosition depth_position(cv::Rect2d& /*target*/);
    void command(TestImageSubscriber::HorizonPosition horizon, TestImageSubscriber::DepthPosition depth);
    void refresh_division_line(void);
    rcl_interfaces::msg::SetParametersResult reflect_param_changes(const std::vector<rclcpp::Parameter>& params);
};