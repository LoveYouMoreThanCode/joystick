#include "commander.h"
#include <iostream>
#include <unistd.h>


extern "C" {
#include "lgpio.h"
}
#include <cassert>
#include <memory>
#include <unordered_map>

int lgTxPwm(int handle, int gpio, float pwmFrequency, float pwmDutyCycle,
            int pwmOffset, int pwmCycles);
#define MOTOR_DRIVE_PWM_FREQ_HZ 1000 /*Hz*/
class Motor {
public:
  Motor(int controller, const std::string &name, uint32_t p1, uint32_t p2)
      : ctl_(controller), name_(name), p1_(p1), p2_(p2) {}
  Motor() : ctl_(-1), name_("unkown"), p1_(UINT32_MAX), p2_(UINT32_MAX) {}
  ~Motor() {}
  int init() {
    // AT8236驱动方式：IN1=1 IN2=1 --> 刹车
    // 默认电平设置为1
    int rc1 = lgGpioClaimOutput(ctl_, 0, p1_, 1);
    int rc2 = lgGpioClaimOutput(ctl_, 0, p2_, 1);
    if (rc1 || rc2) {
      return -1;
    }
    return 0;
  }
  void move_forward(uint32_t speed) {
    std::cout << "motor:" << name_ << " move forward,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_
              << " speed:" << speed << std::endl;
    // AT8236驱动方式：IN1=1 IN2=0 --> 正转
    // lgGpioWrite(ctl_, p1_, 1);
    // lgGpioWrite(ctl_, p2_, 0);
    lgTxPwm(ctl_, p1_, MOTOR_DRIVE_PWM_FREQ_HZ, revise_speed(speed), 0, 0);
    lgTxPwm(ctl_, p2_, 0, 0, 0);
  }
  void move_backward(uint32_t speed) {
    std::cout << "motor:" << name_ << " move backward,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_
              << " speed:" << speed << std::endl;
    // AT8236驱动方式：IN1=0 IN2=1 --> 反转
    // lgGpioWrite(ctl_, p1_, 0);
    // lgGpioWrite(ctl_, p2_, 1);
    lgTxPwm(ctl_, p1_, 0, 0, 0);
    lgTxPwm(ctl_, p2_, MOTOR_DRIVE_PWM_FREQ_HZ, revise_speed(speed), 0, 0);
  }
  void brake() {
    std::cout << "motor:" << name_ << " brake,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_ << std::endl;
    // AT8236驱动方式：IN1=1 IN2=1 --> 刹车
    // lgGpioWrite(ctl_, p1_, 0);
    // lgGpioWrite(ctl_, p2_, 0);
    lgTxPwm(ctl_, p1_, 0, 0, 0);
    lgTxPwm(ctl_, p2_, 0, 0, 0);
  }

private:
  uint32_t revise_speed(uint32_t speed) {
    return std::min(100, std::max(20, speed));
  }

private:
  int ctl_;
  std::string name_;
  uint32_t p1_;
  uint32_t p2_;
};

class Car {
public:
  Car(){};
  ~Car(){};
  int init() {
    ctl_handle_ = lgGpiochipOpen(4);
    assert(ctl_handle_ >= 0);
    uint32_t left_motor_p1 = 17;
    uint32_t left_motor_p2 = 27;
    Motor left(ctl_handle_, "left_rear", left_motor_p1, left_motor_p2);
    int rc = left.init();
    if (rc) {
      return -1;
    }
    uint32_t right_motor_p1 = 23;
    uint32_t right_motor_p2 = 24;
    Motor right(ctl_handle_, "right_rear", right_motor_p1, right_motor_p2);
    rc = right.init();
    if (rc) {
      return rc;
    }
    motors_["left_rear"] = left;
    motors_["right_rear"] = right;

    // add two mor motors here
    uint32_t left_front_p1 = 5;
    uint32_t left_front_p2 = 6;
    Motor left1(ctl_handle_, "left_front", left_front_p1, left_front_p2);
    rc = left1.init();
    if (rc) {
      return -1;
    }
    uint32_t right_front_p1 = 20;
    uint32_t right_front_p2 = 21;
    Motor right1(ctl_handle_, "right_front", right_front_p1, right_front_p2);
    rc = right1.init();
    if (rc) {
      return rc;
    }
    motors_["left_front"] = left1;
    motors_["right_front"] = right1;

    return 0;
  }
  void move_forward() {
    motors_["left_rear"].move_forward(forward_speed_);
    motors_["right_rear"].move_forward(forward_speed_);
    motors_["left_front"].move_forward(forward_speed_);
    motors_["right_front"].move_forward(forward_speed_);
  }
  void move_backward() {
    motors_["left_rear"].move_backward(backward_speed_);
    motors_["right_rear"].move_backward(backward_speed_);
    motors_["left_front"].move_backward(backward_speed_);
    motors_["right_front"].move_backward(backward_speed_);
  }
  void turn_left(bool spin = true) {
    spin = true;
    motors_["left_rear"].move_forward(turn_speed_);
    motors_["left_front"].move_forward(turn_speed_);
    if (spin) {
      motors_["right_rear"].move_backward(turn_speed_);
      motors_["right_front"].move_backward(turn_speed_);
    } else {
      motors_["right_rear"].brake();
      motors_["right_front"].brake();
    }
  }
  void turn_right(bool spin = true) {
    motors_["right_rear"].move_forward(turn_speed_);
    motors_["right_front"].move_forward(turn_speed_);
    if (spin) {
      motors_["left_rear"].move_backward(turn_speed_);
      motors_["left_front"].move_backward(turn_speed_);
    } else {
      motors_["left_rear"].brake();
      motors_["left_front"].brake();
    }
  }
  void brake() {
    motors_["left_rear"].brake();
    motors_["right_rear"].brake();
    motors_["left_front"].brake();
    motors_["right_front"].brake();
  }

private:
  int32_t ctl_handle_;
  std::unordered_map<std::string, Motor> motors_;
  uint32_t forward_speed_{90};
  uint32_t backward_speed_{50};
  uint32_t turn_speed_{30};
};

int main() {
  Car my_car;
  int rc = my_car.init();
  if (rc) {
    std::cout << "failed to init my car, rc:" << rc << std::endl;
    return rc;
  }
  std::unique_ptr<Commander, void (*)(Commander *)> commander(
      make_commander("joystick"), destroy_commander);
  while (true) {
    // 保持一定的控制周期
    lguSleep(0.1);

    // 命令输入提示
    std::cout << std::endl << std::endl;
    std::cout << "...........等待输入指令left(l)/right(r)/forward(f)/"
                 "backward(b)/brake(*)....."
              << std::endl;
    std::string cmd = commander->scan_cmd();

    if (cmd == "left" || cmd == "l") {
      std::cout << "............普通左转100ms............" << std::endl;
      my_car.turn_left();
    } else if (cmd == "right" || cmd == "r") {
      std::cout << "............普通右转100ms............" << std::endl;
      my_car.turn_right();
    } else if (cmd == "forward" || cmd == "f") {
      std::cout << "............前进200ms................" << std::endl;
      my_car.move_forward();
    } else if (cmd == "backward" || cmd == "b") {
      std::cout << "............后退200ms................" << std::endl;
      my_car.move_backward();
    } else {
      std::cout << "............刹车..............." << std::endl;
      my_car.brake();
    }
  }
  // 结束
  return 0;
}
