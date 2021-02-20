#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <pigpiod_if2.h>
#include "../include/motor_controller/test_controller.hpp"

TestController::TestController(
  const rclcpp::NodeOptions& options
): TestController("", options){}

TestController::TestController(
   const std::string& name_space,
   const rclcpp::NodeOptions& options
): Node("test_controller", name_space, options){
    RCLCPP_INFO(this->get_logger(), "constructor");

    init_gpio();

    init_motor();

    motorA->set_pwm(50, 256, 1000);
    motorB->set_pwm(50, 256, 1000);
    foward();
}

TestController::~TestController() {
    RCLCPP_INFO(get_logger(), "destructor");
    stop();
    motorA->set_pwm(0, 256, 1000);
    motorB->set_pwm(0, 256, 1000);
}

int TestController::init_gpio(void) {
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        RCLCPP_ERROR(get_logger(), "pigpio_start error[%d]", pi);
        return 1;
    }
    RCLCPP_INFO(get_logger(), "connect gpiod[%d]", pi);

    return 0;
}

void TestController::init_motor(void) {
    motorA.reset(new DcMotor(pi, APwm, AIn1, AIn2));
    motorA->init();

    motorB.reset(new DcMotor(pi, BPwm, BIn1, BIn2));
    motorB->init();
}

void TestController::foward() {
    motorA->forward();
    motorB->back();
}

void TestController::stop() {
    motorA->stop();
    motorB->stop();
}