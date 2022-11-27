//
// Created by myx on 2022/11/11.
//

#pragma once

#include <serial/serial.h>
#include <memory>
#include <rc_common/hardware_interface/action_interface.h>

namespace rc_hw
{
union F1DataTransCh4
{
  float f_data;
  char ch_data[4];
};
union F6DataTransCh24
{
  char ch_data[24];
  float f_data4[6];
};
enum SerialFlamMark
{
  START,
  FIRST_HEARD,
  SECOND_HEARD,
  DATA,
  FIRST_TAIL
};

struct Action
{
  serial::Serial serial_value;
};
class ActionManager
{
public:
  explicit ActionManager();
  ~ActionManager();

  bool initAction(std::string serialPort, std::shared_ptr<serial::Serial> serial1);
  void readAction(const ros::Time& time, const ros::Duration& period);
  void writeAction(const ros::Time& time, const ros::Duration& period);

  std::vector<rc_hw::Action> actions;
  
  std::vector<rc_control::ActionData> action_data_values;
  std::vector<rc_control::ActionCmd> action_command_values;

private:
  void unPack(const int id, const uint8_t data);
  bool writCmd(const int id, const std::string cmd);
  bool writCmd(const int id, const std::string cmd, const float data);

  std::vector<int> serial_fds_{};
  std::vector<rc_hw::F1DataTransCh4> cmd_trans_;
  std::vector<rc_hw::F6DataTransCh24> data_trans_;
  std::vector<ros::Time> last_write_times_;
  std::vector<rc_hw::SerialFlamMark> serial_flam_marks_;
  std::vector<u_int> serial_date_poses_;
};
}  // namespace rc_hw
