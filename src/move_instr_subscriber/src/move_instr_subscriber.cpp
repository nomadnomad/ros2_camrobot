#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <pigpiod_if2.h>
#include <lifecycle_msgs/msg/state.hpp>
#include "../include/move_instr_subscriber/move_instr_subscriber.hpp"

MoveInstrSubscriber::MoveInstrSubscriber(
    const rclcpp::NodeOptions& options
): MoveInstrSubscriber("", options){}

MoveInstrSubscriber::MoveInstrSubscriber(
    const std::string& namespace_,
    const rclcpp::NodeOptions& options
): rclcpp_lifecycle::LifecycleNode(
    "move_instr_subscriber",
    namespace_,
    rclcpp::NodeOptions(options).use_intra_process_comms(false)) {
    
    RCLCPP_INFO(this->get_logger(), "constructor");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_configure(const rclcpp_lifecycle::State &) {

    init_subscription();
    init_gpio();
    init_motor();


    motorA->set_pwm(50, 256, 1000);
    motorB->set_pwm(50, 256, 1000);
    forward();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_activate(const rclcpp_lifecycle::State &) {

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_deactivate(const rclcpp_lifecycle::State &) {

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_cleanup(const rclcpp_lifecycle::State &) {

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_shutdown(const rclcpp_lifecycle::State &) {
    finish();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MoveInstrSubscriber::init_subscription(void) {
  subscription = create_subscription<example_interfaces::msg::String>(
    "move_instr",
    rclcpp::QoS(10),
    std::bind(&MoveInstrSubscriber::subscribe_move_instr, this, std::placeholders::_1)
  );
}

int MoveInstrSubscriber::init_gpio(void) {
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        RCLCPP_ERROR(get_logger(), "pigpio_start error[%d]", pi);
        return 1;
    }
    RCLCPP_INFO(get_logger(), "connect gpiod[%d]", pi);

    return 0;
}

void MoveInstrSubscriber::init_motor(void) {
    motorA.reset(new DcMotor(pi, APwm, AIn1, AIn2));
    motorA->init();

    motorB.reset(new DcMotor(pi, BPwm, BIn1, BIn2));
    motorB->init();
}

void MoveInstrSubscriber::finish(void) {
    motorA->shutdown();
    motorB->shutdown();
}

void MoveInstrSubscriber::subscribe_move_instr(const example_interfaces::msg::String::SharedPtr instr) {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }
    RCLCPP_INFO(get_logger(), "current_state[%d][%s] instr[%s]",
      this->get_current_state().id(),
      this->get_current_state().label().c_str(), 
      instr->data.c_str());

    if (instr->data == "forward") {
        forward();
    } else if (instr->data == "stop") {
        stop();
    }
}

void MoveInstrSubscriber::forward(void) {
    motorA->forward();
    motorB->back();
}

void MoveInstrSubscriber::stop() {
    motorA->stop();
    motorB->stop();
}