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

#include "joystick.hh"
#include <cassert>
#include <unistd.h>


class JoystickDirCommand {
public:
  JoystickDirCommand(std::string path) : js_(path) {}
  ~JoystickDirCommand() {}
  std::string scan_cmd() {
    assert(js_.isFound());
    JoystickEvent event;
    while (js_.sample(&event)) {
      if (!event.isAxis()) {
        continue;
      }
      if (event.number == 0) {
        x_ = event.value;
      } else if (event.number == 1) {
        y_ = event.value;
      }
    }
    return make_cmd();
  }

private:
  std::string make_cmd() {
    if (y_ == 0) {
      return "brake";
    }
    if (y_ < 0) {
      return "backward";
    }
    if (x_ == 0) {
      return "forward";
    } else if (x_ < 0) {
      return "left";
    } else {
      return "right";
    }
  }

private:
  int x_{0};
  int y_{0};
  Joystick js_;
};

int main(int argc, char **argv) {
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound()) {
    printf("open failed.\n");
    exit(1);
  }

  while (true) {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event)) {
      if (event.isButton()) {
        printf("Button %u is %s\n", event.number,
               event.value == 0 ? "up" : "down");
      } else if (event.isAxis()) {
        printf("Axis %u is at position %d\n", event.number, event.value);
      }
    }
  }

  JoystickDirCommand js_cmd("/dev/input/js0");
  while (true) {
    std::string cmd = js_cmd.scan_cmd();
    sleep(1);
  }
}
