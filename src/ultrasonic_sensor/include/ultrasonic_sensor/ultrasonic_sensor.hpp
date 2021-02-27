#include <chrono> 
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <example_interfaces/msg/int32.hpp>
#include "ultrasonic_sensor_msgs/msg/ultrasonic.hpp"

class UltrasonicSensor : public rclcpp_lifecycle::LifecycleNode {
  using SensorTuple = std::tuple<std::chrono::system_clock::time_point, double>;

  public:
    UltrasonicSensor(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    UltrasonicSensor(
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

  private:
    rclcpp_lifecycle::LifecyclePublisher<ultrasonic_sensor_msgs::msg::Ultrasonic>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr ultrasonic_timer_;
    int pi;
    const int GPIO_TRIG = 12;
    const int GPIO_ECHO = 6;

    void publish_ultrasonic_sensor();  
    double sensor_distance();
    bool init();
    std::chrono::system_clock::time_point readWhileEchoOff();
    SensorTuple dist_ave();
    std::shared_ptr<ultrasonic_sensor_msgs::msg::Ultrasonic> create_data(
        UltrasonicSensor::SensorTuple& sensorTuple);
};
