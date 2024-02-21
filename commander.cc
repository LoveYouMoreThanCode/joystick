// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016
#include "commander.h"
#include "joystick.hh"
#include <cassert>

class JsCommander: public Commander {
public:
  JsCommander(std::string path) : js_(path) {
    assert(js_.isFound());
  }
  ~JsCommander() {}
  std::string scan_cmd() override{
    JoystickEvent event;
    while (js_.sample(&event)) {
      if (!event.isAxis()) {
        continue;
      }
      if (event.number == 4) {
        x_ = event.value;
      } else if (event.number == 5) {
        y_ = event.value;
      }
    }
    printf("x=%d y=%d\n",x_,y_);
    return make_cmd();
  }

private:
  std::string make_cmd() {
    if (x_ == 0 && y_== 0) {
      return "brake";
    }
    if (x_ < 0) {
      return "forward";
    }
    if (y_ == 0) {
      return "backward";
    } else if (y_ < 0) {
      return "right";
    } else {
      return "left";
    }
  }

private:
  int x_{0};
  int y_{0};
  Joystick js_;
};

class TerminalCommander :public Commander {
  public:
  TerminalCommander(){}
  ~TerminalCommander(){
  }
  std::string scan_cmd()override {
    std::string cmd;
    std::cin>>cmd;
    return cmd;
  }
};

Commander *make_commander(std::string type) {
  if (type == "joystick") {
    return new JsCommander("/dev/input/js0");
  }else if (type == "terminal") {
    return new TerminalCommander();
  }
  return nullptr;
}
void destroy_commander(Commander *cmd) {
  delete cmd;
}