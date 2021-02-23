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
    RCLCPP_INFO(get_logger(), "on_configure start");

    if (init_gpio() != 0) {
        RCLCPP_INFO(get_logger(), "on_configure failure");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    init_motor();
    init_subscription();

    RCLCPP_INFO(get_logger(), "on_configure normal end");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_activate start");

    RCLCPP_INFO(get_logger(), "on_activate normal end");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_deactivate start");

    RCLCPP_INFO(get_logger(), "on_deactivate normal end");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_cleanup start");

    RCLCPP_INFO(get_logger(), "on_cleanup normal end");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveInstrSubscriber::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_shutdown start");

    finish();

    RCLCPP_INFO(get_logger(), "on_shutdown normal end");
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
    motorR.reset(new DcMotor(pi, APwm, AIn1, AIn2));
    int initResultMotorR = motorR->init();
    RCLCPP_INFO(get_logger(), "init MotorR result[%d]", initResultMotorR);
    motorR->stop();
    motorR->set_pwm(50, 256, 1000);

    motorL.reset(new DcMotor(pi, BPwm, BIn1, BIn2));
    int initResultMotorL = motorL->init();
    RCLCPP_INFO(get_logger(), "init MotorL result[%d]", initResultMotorL);
    motorL->stop();
    motorL->set_pwm(50, 256, 1000);
}

void MoveInstrSubscriber::finish(void) {
    motorR->shutdown();
    motorL->shutdown();
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
    } else if (instr->data == "back") {
        back();
    } else if (instr->data == "turn_right") {
        turn_right();
    } else if (instr->data == "turn_left") {
        turn_left();
    }
}

void MoveInstrSubscriber::forward(void) {
    motorR->forward();
    motorL->back();
}

void MoveInstrSubscriber::back(void) {
    motorR->back();
    motorL->forward();
}

void MoveInstrSubscriber::stop(void) {
    motorR->stop();
    motorL->stop();
}

void MoveInstrSubscriber::turn_right(void) {
    motorR->back();
    motorL->back();
}

void MoveInstrSubscriber::turn_left(void) {
    motorR->forward();
    motorL->forward();
}