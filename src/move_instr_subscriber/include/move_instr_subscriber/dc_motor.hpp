#pragma once

class DcMotor {
  public:
    DcMotor(int pi, int pwm, int in1, int in2);
    void init(void);
    void shutdown(void);
    void set_pwm(unsigned duty, unsigned range, unsigned freq);
    void forward(void);
    void back(void);
    void brake(void);
    void stop(void);
  private:
    int pi;
    int pwm;
    int in1;
    int in2;
};