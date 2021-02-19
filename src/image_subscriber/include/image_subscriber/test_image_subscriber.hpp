#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

class TestImageSubscriber : public rclcpp_lifecycle::LifecycleNode {
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    void init(void);    
    void subscribe_image(const sensor_msgs::msg::Image::SharedPtr image);

  public:
    TestImageSubscriber(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    TestImageSubscriber(
      const std::string& name_space,
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