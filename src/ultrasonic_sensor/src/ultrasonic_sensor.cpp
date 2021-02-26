#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <chrono> 
#include <pigpiod_if2.h>
#include "../include/ultrasonic_sensor/ultrasonic_sensor.hpp"

UltrasonicSensor::UltrasonicSensor(
  const rclcpp::NodeOptions& options
)
: UltrasonicSensor("", options){}

UltrasonicSensor::UltrasonicSensor(
  const std::string& namespace_,
  const rclcpp::NodeOptions& options
)
: rclcpp_lifecycle::LifecycleNode("ultrasonic_sensor_node", 
      namespace_,
      rclcpp::NodeOptions(options).use_intra_process_comms(false)) {

    RCLCPP_INFO(get_logger(), "constructor");
}

double UltrasonicSensor::sensor_distance() {
  using namespace std::chrono_literals;

  rclcpp::WallRate trig_rate(15us);
  rclcpp::WallRate read_rate(5us);
  std::chrono::system_clock::time_point  start, end;

  gpio_write(pi, GPIO_TRIG, 1);
  trig_rate.sleep();
  gpio_write(pi, GPIO_TRIG, 0);

/*
  start = std::chrono::system_clock::now(); // 計測開始時間
  while (gpio_read(pi, GPIO_ECHO) == 0) {
    start = std::chrono::system_clock::now(); // 計測開始時間
  }
*/
  int echo_off;
  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    goto ECHO_ON;
  }

ECHO_ON:
  end = std::chrono::system_clock::now();  // 計測終了時間
  while (gpio_read(pi, GPIO_ECHO) == 1) {
    read_rate.sleep();
    end = std::chrono::system_clock::now();  // 計測終了時間
  }

  auto duration = end - start;
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  double distance = (usec / 1000000.0) * 34000.0 / 2.0;
  distance = round(distance * 10.0) / 10.0;

  //RCLCPP_INFO(get_logger(), "publish_ultrasonic_sensor count[%lfcm]", distance);
  return distance;
}

void UltrasonicSensor::publish_ultrasonic_sensor() {
  double distance;

  distance = sensor_distance();
  distance += sensor_distance();
  distance += sensor_distance();
  distance /= 3.0;
  distance = round(distance * 10.0) / 10.0;

  RCLCPP_INFO(get_logger(), "publish_ultrasonic_sensor distance[%lfcm]", distance);

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_configure(const rclcpp_lifecycle::State &) {
  using namespace std::chrono_literals;

  pi = pigpio_start(NULL, NULL);
  if (pi < 0) {
    RCLCPP_ERROR(get_logger(), "pigpio_start error[%d]", pi);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
    RCLCPP_INFO(get_logger(), "connect gpiod[%d]", pi);
  RCLCPP_INFO(this->get_logger(), "on_configure");
  set_mode(pi, GPIO_TRIG, PI_OUTPUT);
  set_mode(pi, GPIO_ECHO, PI_INPUT);
  
  ultrasonic_timer_ = create_wall_timer(
    1000ms, 
    std::bind(&UltrasonicSensor::publish_ultrasonic_sensor, this));
  ultrasonic_timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_activate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(this->get_logger(), "on_activate");

  ultrasonic_timer_->reset();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(this->get_logger(), "on_deactivate");

  ultrasonic_timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_cleanup(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(this->get_logger(), "on_cleanup");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_shutdown(const rclcpp_lifecycle::State &){

  RCLCPP_INFO(this->get_logger(), "on_shutdown");

  pigpio_stop(pi);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
