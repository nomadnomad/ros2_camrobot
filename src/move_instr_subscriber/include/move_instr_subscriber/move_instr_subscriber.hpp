#include <example_interfaces/msg/string.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "dc_motor.hpp"

enum class Dir {
  Left,
  Right,
};

class MoveInstrSubscriber : public rclcpp_lifecycle::LifecycleNode {
  private:
    const int APwm = 17;
    const int AIn1 = 22;
    const int AIn2 = 27;
    const int BPwm = 26;
    const int BIn1 = 13;
    const int BIn2 = 19;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscription;
    int pi;
    std::unique_ptr<DcMotor> motorR;
    std::unique_ptr<DcMotor> motorL;

    void subscribe_move_instr(const example_interfaces::msg::String::SharedPtr instr);
    void init_subscription(void);
    int init_gpio(void);
    void init_motor(void);
    void finish(void);
    
    void forward(void);
    void back(void);
    void stop(void);
    void turn_right(void);
    void turn_left(void);

  public:
    MoveInstrSubscriber(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    MoveInstrSubscriber(
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