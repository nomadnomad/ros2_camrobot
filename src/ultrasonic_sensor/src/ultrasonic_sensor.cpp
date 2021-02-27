#include <chrono> 
#include <vector>
#include <numeric>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pigpiod_if2.h>
#include "ultrasonic_sensor_msgs/msg/ultrasonic.hpp"
#include "../include/ultrasonic_sensor/ultrasonic_sensor.hpp"

UltrasonicSensor::UltrasonicSensor(
  const rclcpp::NodeOptions& options
)
: UltrasonicSensor("", options){}

UltrasonicSensor::UltrasonicSensor(
    const std::string& namespace_,
    const rclcpp::NodeOptions& options
):rclcpp_lifecycle::LifecycleNode("ultrasonic_sensor_node", 
    namespace_,
    rclcpp::NodeOptions(options).use_intra_process_comms(false)) {

    RCLCPP_INFO(get_logger(), "constructor");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_configure(const rclcpp_lifecycle::State &) {
  if (!init()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_activate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(this->get_logger(), "on_activate");

  publisher_->on_activate();
  ultrasonic_timer_->reset();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UltrasonicSensor::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(this->get_logger(), "on_deactivate");

  publisher_->on_deactivate();
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

  publisher_->on_deactivate();
  ultrasonic_timer_->cancel();
  pigpio_stop(pi);
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool UltrasonicSensor::init() {
  using namespace std::chrono_literals;

  pi = pigpio_start(NULL, NULL);
  if (pi < 0) {
    RCLCPP_ERROR(get_logger(), "pigpio_start error[%d]", pi);
    return false;
  }
  RCLCPP_INFO(get_logger(), "connect gpiod[%d]", pi);
  RCLCPP_INFO(this->get_logger(), "on_configure");
  set_mode(pi, GPIO_TRIG, PI_OUTPUT);
  set_mode(pi, GPIO_ECHO, PI_INPUT);

  publisher_ = this->create_publisher<ultrasonic_sensor_msgs::msg::Ultrasonic>(
      "ultrasonic_sensor",
      rclcpp::QoS(10));

  ultrasonic_timer_ = create_wall_timer(
    100ms, 
    std::bind(&UltrasonicSensor::publish_ultrasonic_sensor, this));
  ultrasonic_timer_->cancel();

  return true;
}

std::chrono::system_clock::time_point UltrasonicSensor::readWhileEchoOff() {
  int echo_off;
  std::chrono::system_clock::time_point start = 
    std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  start = std::chrono::system_clock::now(); // 計測開始時間
  echo_off = gpio_read(pi, GPIO_ECHO);
  if (echo_off == 1) {
    return start;
  }

  return start;
}

double UltrasonicSensor::sensor_distance() {
  using namespace std::chrono_literals;

  rclcpp::WallRate trig_rate(15us);
  rclcpp::WallRate read_rate(5us);

  gpio_write(pi, GPIO_TRIG, 1);
  trig_rate.sleep();
  gpio_write(pi, GPIO_TRIG, 0);

  // whileでgpio_read(pi, GPIO_ECHO)を呼ぶと近接している時に停止するので
  // とりあえずgpiod_read(pi, GPIO_ECHO)をシリアルに呼んでいます
/*
  start = std::chrono::system_clock::now(); // 計測開始時間
  while (gpio_read(pi, GPIO_ECHO) == 0) {
    start = std::chrono::system_clock::now(); // 計測開始時間
  }
*/
  // 計測開始時間
  std::chrono::system_clock::time_point start = readWhileEchoOff();

  // 計測終了時間
  std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
  while (gpio_read(pi, GPIO_ECHO) == 1) {
    read_rate.sleep();
    end = std::chrono::system_clock::now();
  }

  auto duration = end - start;
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  double distance = (usec / 1000000.0) * 34000.0 / 2.0;
  distance = round(distance * 10.0) / 10.0;

  return distance;
}

UltrasonicSensor::SensorTuple UltrasonicSensor::dist_ave() {
  std::vector<double> dist;
  for (int i = 0; i < 3; i++) {
    dist.push_back(sensor_distance());
  }
  double distance_ave = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();
  distance_ave = round(distance_ave * 10.0) / 10.0;

  auto now = std::chrono::system_clock::now();

  RCLCPP_INFO(get_logger(), "publish_ultrasonic_sensor distance[%4.1lfcm]", distance_ave);
  return std::make_tuple(now, distance_ave);
}

std::shared_ptr<ultrasonic_sensor_msgs::msg::Ultrasonic>
UltrasonicSensor::create_data(UltrasonicSensor::SensorTuple& sensorTuple) {
  auto now = std::get<0>(sensorTuple);
  auto dist = std::get<1>(sensorTuple);
  
  auto nanosecs=
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()
    );
  auto sec = nanosecs.count() / 1000000000;
  auto nanosec= nanosecs.count() % 1000000000;

  auto msg = std::make_shared<ultrasonic_sensor_msgs::msg::Ultrasonic>();
  msg->header.stamp.sec = sec;
  msg->header.stamp.nanosec = nanosec;
  msg->header.frame_id = "ultrasonic";
  msg->dist = dist;

  return msg;
}

void UltrasonicSensor::publish_ultrasonic_sensor() {
  auto sensorTuple = dist_ave();
  auto msg = create_data(sensorTuple);
  if (publisher_->is_activated()) {
    publisher_->publish(*msg);
  } else {
    RCLCPP_WARN(
        get_logger(), 
        "Lifecycle publisher is currently inactive. Messages are not published.");
  }
}
 
