#include <iostream>
#include <unistd.h>
#include "commander.h"

extern "C" {
#include "lgpio.h"
}
#include <cassert>
#include <memory>
#include <unordered_map>

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
  void move_forward() {
    std::cout << "motor:" << name_ << " move forward,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_ << std::endl;
    // AT8236驱动方式：IN1=1 IN2=0 --> 正转
    lgGpioWrite(ctl_, p1_, 1);
    lgGpioWrite(ctl_, p2_, 0);
  }
  void move_backward() {
    std::cout << "motor:" << name_ << " move backward,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_ << std::endl;
    // AT8236驱动方式：IN1=0 IN2=1 --> 反转
    lgGpioWrite(ctl_, p1_, 0);
    lgGpioWrite(ctl_, p2_, 1);
  }
  void brake() {
    std::cout << "motor:" << name_ << " brake,"
              << " ctl:" << ctl_ << " p1:" << p1_ << " p2:" << p2_ << std::endl;
    // AT8236驱动方式：IN1=1 IN2=1 --> 刹车
    lgGpioWrite(ctl_, p1_, 1);
    lgGpioWrite(ctl_, p2_, 1);
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
    Motor left(ctl_handle_, "left", left_motor_p1, left_motor_p2);
    int rc = left.init();
    if (rc) {
      return -1;
    }
    uint32_t right_motor_p1 = 23;
    uint32_t right_motor_p2 = 24;
    Motor right(ctl_handle_, "right", right_motor_p1, right_motor_p2);
    rc = right.init();
    if (rc) {
      return rc;
    }
    motors_["left"] = left;
    motors_["right"] = right;
    return 0;
  }
  void move_forward() {
    motors_["left"].move_forward();
    motors_["right"].move_forward();
  }
  void move_backward() {
    motors_["left"].move_backward();
    motors_["right"].move_backward();
  }
  void turn_left(bool spin = false) {
    motors_["left"].move_forward();
    if (spin) {
      motors_["right"].move_backward();
    } else {
      motors_["right"].brake();
    }
  }
  void turn_right(bool spin = false) {
    motors_["right"].move_forward();
    if (spin) {
      motors_["left"].move_backward();
    } else {
      motors_["left"].brake();
    }
  }
  void brake() {
    motors_["left"].brake();
    motors_["right"].brake();
  }

private:
  int32_t ctl_handle_;
  std::unordered_map<std::string, Motor> motors_;
};


int main() {
  Car my_car;
  int rc = my_car.init();
  if (rc) {
    std::cout<<"failed to init my car, rc:"<<rc<<std::endl;
    return rc;
  }
  std::unique_ptr<Commander, void (*)(Commander *)> commander(
      make_commander("joystick"), destroy_commander);
  while (true) {
    //保持一定的控制周期
    lguSleep(0.1);

    //命令输入提示
    std::cout << std::endl << std::endl;
    std::cout
        << "...........等待输入指令left(l)/right(r)/forward(f)/backward(b)/brake(*)....."
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
  //结束
  return 0;
}
