//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls
               
#pragma once

#include <XmlRpcValue.h>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <rc_common/hardware_interface/gpio_interface.h>
#include <ros/ros.h>
#include <string>

namespace rc_hw
{
class GpioManager
{
public:
  explicit GpioManager();
  ~GpioManager();

  void setGpioDirection(rc_control::GpioData gpioData);
  void readGpio();
  void writeGpio();

  std::vector<rc_control::GpioData> gpio_state_values;
  std::vector<rc_control::GpioData> gpio_command_values;
};
}  // namespace rc_hw
