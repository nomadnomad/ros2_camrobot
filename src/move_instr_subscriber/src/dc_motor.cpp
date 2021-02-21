#include <pigpiod_if2.h>
#include "../include/move_instr_subscriber/dc_motor.hpp"

DcMotor::DcMotor(int pi, int pwm, int in1, int in2) {
  this->pi = pi;
  this->pwm = pwm;
  this->in1 = in1;
  this->in2 = in2;
}

void DcMotor::init() {
    set_mode(pi, pwm, PI_OUTPUT);
    set_PWM_dutycycle(pi, pwm, 0);

    set_mode(pi, in1, PI_OUTPUT);
    gpio_write(pi, in1, 0);

    set_mode(pi, in2, PI_OUTPUT);
    gpio_write(pi, in2, 0);
}

void DcMotor::shutdown() {
    stop();
    set_pwm(0, 256, 1000);
}

void DcMotor::set_pwm(unsigned duty, unsigned range, unsigned freq) {
    set_PWM_range(pi, pwm, range);
    set_PWM_frequency(pi, pwm, freq);
    set_PWM_dutycycle(pi, pwm, (int)(range*duty/100.0));
}

void DcMotor::forward() {
    gpio_write(pi, in1, 1);
    gpio_write(pi, in2, 0);
}

void DcMotor::back() {
    gpio_write(pi, in1, 0);
    gpio_write(pi, in2, 1);
}

void DcMotor::brake() {
    gpio_write(pi, in1, 1);
    gpio_write(pi, in2, 1);
}

void DcMotor::stop() {
    gpio_write(pi, in1, 0);
    gpio_write(pi, in2, 0);
}