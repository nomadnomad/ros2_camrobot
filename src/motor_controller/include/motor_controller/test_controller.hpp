#include <rclcpp/rclcpp.hpp>
#include "dc_motor.hpp"

enum class Dir {
  Left,
  Right,
};

class TestController : public rclcpp::Node{
  public:
    TestController(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    TestController(
      const std::string& name_space,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    ~TestController();

  private:
    const int APwm = 17;
    const int AIn1 = 22;
    const int AIn2 = 27;
    const int BPwm = 26;
    const int BIn1 = 13;
    const int BIn2 = 19;
    int pi;
    std::unique_ptr<DcMotor> motorA;
    std::unique_ptr<DcMotor> motorB;
    int init_gpio(void);
    void init_motor(void);
    void foward(void);
    void stop(void);
};