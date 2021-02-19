#include <rclcpp/rclcpp.hpp>
#include "../include/moter_controller/test_controller.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestController>());
  rclcpp::shutdown();
  return 0;
}