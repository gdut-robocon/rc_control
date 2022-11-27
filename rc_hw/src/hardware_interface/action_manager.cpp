//
// Created by myx on 2022/11/12.
//
#include <rc_hw/hardware_interface/action_manager.h>
#include <rc_common/math_utilities.h>

#include <fcntl.h> /* File control definitions */
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace rc_hw
{
ActionManager::ActionManager()
{
}

ActionManager::~ActionManager()
{
  for (auto& item : serial_fds_)
  {
    close(item);
  }
}

bool ActionManager::initAction(std::string serialPort, std::shared_ptr<serial::Serial> serial1)
{
  char* port = (char*)serialPort.data();
  int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd == -1)
  {
    ROS_ERROR_STREAM("Unable to open serial of action. Action name:" << serialPort);
    return false;
  }
  struct termios options
  {
  };

  tcflush(fd, TCIOFLUSH);
  // tcgetattr(fd, &options);
  ioctl(fd, TCGETS, &options);
  // cfsetospeed(&options, B115200);
  // cfsetispeed(&options, B115200);
  options.c_ispeed = B115200;
  options.c_ospeed = B115200;

  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_cflag &= ~PARENB;
  options.c_iflag &= ~INPCK;

  options.c_cflag &= ~CSTOPB;

  // update configuration
  // tcsetattr(fd, TCSANOW, &options);
  ioctl(fd, TCSETS, &options);
  serial_fds_.push_back(fd);
  ROS_INFO_STREAM("Successful to open " << serialPort << " port.");

  rc_hw::F1DataTransCh4 cmd_tran_;
  rc_hw::F6DataTransCh24 data_tran_;
  ros::Time last_writ_time_ = ros::Time::now();
  rc_hw::SerialFlamMark serial_flam_mark_;
  u_int serial_data_pose_ = 0;
  cmd_tran_.f_data = 0.0;
  for (int i = 0; i < 6; i++)
  {
    data_tran_.f_data4[i] = 0.0;
  }
  serial_flam_mark_ = START;
  cmd_trans_.push_back(cmd_tran_);
  data_trans_.push_back(data_tran_);
  last_write_times_.push_back(last_writ_time_);
  serial_flam_marks_.push_back(serial_flam_mark_);
  serial_date_poses_.push_back(serial_data_pose_);

  return true;
}

void ActionManager::readAction(const ros::Time& time, const ros::Duration& period)
{
  for (int id = 0; id < serial_fds_.size(); ++id)
  {
    if (action_command_values[id].calibration_state)
    {
      uint8_t buffer[1];
      uint8_t num = 1;
      for (int i = 0; i < 28; ++i)
      {
        if (read(serial_fds_[id], buffer, 1) == 0)
        {
          break;
        }
        unPack(id, buffer[0]);
      }
    }
  }
}

void ActionManager::writeAction(const ros::Time& time, const ros::Duration& period)
{
  for (int id = 0; id < serial_fds_.size(); id++)
  {
    // calibration_state default id True.
    if (!action_command_values[id].calibration_state)
    {
      // action calibration need cost 15min in absolute stillness
      //      writCmd(id, "ACTR");
      ROS_WARN_STREAM("Please keep this " << action_data_values[id].name
                                          << " in absolute stillness and continuous 15min.");
      action_command_values[id].calibration_state = true;
    }

    if (action_command_values[id].reset_state)
    {
      // action calibration need cost 15min in absolute stillness
      if (writCmd(id, "ACT0"))
      {
        ROS_INFO_STREAM("Rest " << action_data_values[id].name << " , starting over with data fusion.");
        action_command_values[id].calibration_state = false;
      }
    }

    if (action_command_values[id].update_yaw_state)
    {
      if (writCmd(id, "ACTZ", radToAng(action_command_values[id].update_yaw)))
      {
        ROS_INFO_STREAM("Yaw_angle of " << action_data_values[id].name << " has updated to "
                                        << action_command_values[id].update_yaw << ".");
        action_command_values[id].update_yaw_state = false;
      }
    }

    if (action_command_values[id].update_x_state)
    {
      if (writCmd(id, "ACTX", action_command_values[id].update_x * 1000.0))
      {
        ROS_INFO_STREAM("Pose_x of " << action_data_values[id].name << " has updated to "
                                     << action_command_values[id].update_x << ".");
        action_command_values[id].update_x_state = false;
      }
    }

    if (action_command_values[id].update_y_state)
    {
      if (writCmd(id, "ACTY", action_command_values[id].update_y * 1000.0))
      {
        ROS_INFO_STREAM("Pose_y of " << action_data_values[id].name << " has updated to "
                                     << action_command_values[id].update_y << ".");
        action_command_values[id].update_y_state = false;
      }
    }
  }
}

void ActionManager::unPack(const int id, const uint8_t data)
{
  switch (serial_flam_marks_[id])
  {
    case START:
      if (data == 0x0d)
      {
        serial_flam_marks_[id] = FIRST_HEARD;
      }
      else
      {
        serial_flam_marks_[id] = START;
      }
      break;
    case FIRST_HEARD:
      if (data == 0x0a)
      {
        serial_date_poses_[id] = 0;
        serial_flam_marks_[id] = SECOND_HEARD;
      }
      else if (data == 0x0d)
      {
        serial_flam_marks_[id] = FIRST_HEARD;
      }
      else
      {
        serial_flam_marks_[id] = START;
      }
      break;
    case SECOND_HEARD:
      data_trans_[id].ch_data[serial_date_poses_[id]] = data;
      serial_date_poses_[id]++;
      if (serial_date_poses_[id] >= 24)
      {
        serial_date_poses_[id] = 0;
        serial_flam_marks_[id] = DATA;
      }
      break;
    case DATA:
      if (data == 0x0a)
      {
        serial_flam_marks_[id] = FIRST_TAIL;
      }
      else
      {
        serial_flam_marks_[id] = START;
      }
      break;
    case FIRST_TAIL:
      if (data == 0x0d)  // there state is SECOND_TAIL
      {
        action_data_values[id].yaw_angle = angToRad(data_trans_[id].f_data4[0]);
        action_data_values[id].pitch_angle = angToRad(data_trans_[id].f_data4[1]);
        action_data_values[id].roll_angle = angToRad(data_trans_[id].f_data4[2]);
        action_data_values[id].pose_x = data_trans_[id].f_data4[3] / 1000.0;
        action_data_values[id].pose_y = data_trans_[id].f_data4[4] / 1000.0;
        action_data_values[id].yaw_acc = angToRad(data_trans_[id].f_data4[5]);
      }
      serial_flam_marks_[id] = START;
      break;
    default:
      serial_flam_marks_[id] = START;
      break;
  }
}

bool ActionManager::writCmd(const int id, std::string cmd)
{
  if (write(serial_fds_[id], (char*)cmd.data(), cmd.length()) != -1)
  {
    ROS_DEBUG_STREAM("Successful write command to " << action_data_values[id].name << ".");
  }
  else
  {
    ROS_DEBUG_STREAM("Failed write command to " << action_data_values[id].name << ".");
    return false;
  }
  return true;
}

bool ActionManager::writCmd(const int id, const std::string cmd, const float data)
{
  cmd_trans_[id].f_data = data;
  // update cmd need to delay 10ms
  if (ros::Time::now() - last_write_times_[id] >= ros::Duration(0.01))
  {
    if (write(serial_fds_[id], (char*)cmd.data(), cmd.length()) != -1 ||
        write(serial_fds_[id], (char*)cmd_trans_[id].ch_data, cmd.length()) != -1)
    {
      ROS_DEBUG_STREAM("Successful write command to " << action_data_values[id].name << ".");
    }
    else
    {
      ROS_DEBUG_STREAM("Failed write command to " << action_data_values[id].name << ".");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM("action " << action_data_values[id].name << " is updating. Please try to again past 10ms.");
  }
  return true;
}
}  // namespace rc_hw
