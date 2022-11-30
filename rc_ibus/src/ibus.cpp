//
// Created by myx on 2022/11/29.
//

#include <rc_ibus/ibus.h>
#include <ros/ros.h>

#include <fcntl.h> /* File control definitions */
#include <termios.h>
#include <unistd.h>

extern "C" {
extern int ioctl(int __fd, unsigned long int __request, ...) throw();
}

namespace rc_ibus
{
IBus::IBus()
{
}
IBus::~IBus()
{
}
void IBus::init(const char* serial)
{
  int fd = open(serial, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd == -1)
  {
    ROS_ERROR("[rt_ibus] Unable to open ibus\n");
    return;
  }
  struct termios options
  {
  };

  // Even parity(115200 8N1):
  if (tcgetattr(fd, &options) != 0)
  {
    perror("SetupSerial 1");
  }
  bzero(&options, sizeof(options));
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSIZE;

  options.c_cflag |= CS8;

  options.c_cflag &= ~PARENB;

  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag &= ~CSTOPB;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);
  if ((tcsetattr(fd, TCSANOW, &options)) != 0)
  {
    perror("com set error");
  }
  ROS_INFO_STREAM("Successful to open " << serial << " port.");
  port_ = fd;
}

void IBus::read()
{
  uint8_t read_byte;
  int timeout = 0;  // time out of one package
  while (timeout < 5)
  {
    if (count == 3)
    {
      count = 0;
      return;
    }
    // Read a byte //
    size_t n = ::read(port_, &read_byte, sizeof(read_byte));

    if (n == 0)
    {
      timeout++;
    }
    else if (n == 1)
    {
      unpack(read_byte);
    }
  }
}

void IBus::unpack(const uint8_t data)
{
  switch (count)
  {
    case 0:
      if (data == 0x20)
      {
        data_i = 0;
        buff_[data_i] = 0x20;
        count++;
      }
      else
      {
        count = 0;
      }
      break;
    case 1:
      if (data == 0x40)
      {
        data_i = 1;
        buff_[data_i] = 0x40;
        count++;
      }
      else
      {
        count = 0;
      }
      break;
    case 2:
      data_i++;
      buff_[data_i] = data;
      i_bus_data_.checksum_ibus = (uint16_t)(buff_[30] << 8 | buff_[31]);
      if (data_i >= 31)
      {
        count++;
        data_i = 0;

        for (int i = 0; i < 10; ++i)
        {
          i_bus_data_.ch[i] = (uint16_t)((buff_[i * 2 + 3] & 0x0F) << 8 | buff_[i * 2 + 2]);
        }
        is_update_ = true;
      }
      break;
    default:
      count = 0;
      break;
  }
}

void IBus::getData(rc_msgs::IbusData* i_bus_cmd)
{
  if (is_update_)
  {
    i_bus_cmd->stamp = ros::Time::now();

    // ch_r_*
    for (int i = 0; i < 4; ++i)
    {
      if (abs(i_bus_data_.ch[i] - 1500) <= 10)
        i_bus_data_.ch[i] = 1500;

      if (abs(i_bus_data_.ch[i] - 1500) > 500)
      {
        return;
      }
    }
    i_bus_cmd->ch_r_x = static_cast<double>((i_bus_data_.ch[0] - 1500.0) / 500.0);
    i_bus_cmd->ch_r_y = static_cast<double>((i_bus_data_.ch[1] - 1500.0) / 500.0);
    i_bus_cmd->ch_l_y = static_cast<double>((i_bus_data_.ch[2] - 1500.0) / 500.0);
    i_bus_cmd->ch_l_x = static_cast<double>((i_bus_data_.ch[3] - 1500.0) / 500.0);

    // i_bus_cmd->sw_a i_bus_data_.ch[4]
    if (abs(i_bus_data_.ch[4] - 1000) < 5)  // 1000
    {
      i_bus_cmd->sw_a = i_bus_cmd->UP;
    }
    else if (abs(i_bus_data_.ch[4] - 2000) < 5)  // 2000
    {
      i_bus_cmd->sw_a = i_bus_cmd->DOWN;
    }

    // i_bus_cmd->sw_a i_bus_data_.ch[5]
    if (abs(i_bus_data_.ch[5] - 1000) < 5)
    {
      i_bus_cmd->sw_b = i_bus_cmd->UP;
    }
    else if (abs(i_bus_data_.ch[5] - 1500) < 5)
    {
      i_bus_cmd->sw_b = i_bus_cmd->MID;
    }
    else if (abs(i_bus_data_.ch[5] - 2000) < 5)
    {
      i_bus_cmd->sw_b = i_bus_cmd->DOWN;
    }

    // i_bus_cmd->sw_c i_bus_data_.ch[6]
    if (abs(i_bus_data_.ch[6] - 1000) < 5)
    {
      i_bus_cmd->sw_c = i_bus_cmd->UP;
    }
    else if (abs(i_bus_data_.ch[6] - 1500) < 5)
    {
      i_bus_cmd->sw_c = i_bus_cmd->MID;
    }
    else if (abs(i_bus_data_.ch[6] - 2000) < 5)
    {
      i_bus_cmd->sw_c = i_bus_cmd->DOWN;
    }

    // i_bus_cmd->sw_d i_bus_data_.ch[7]
    if (abs(i_bus_data_.ch[7] - 1000) < 5)
    {
      i_bus_cmd->sw_d = i_bus_cmd->UP;
    }
    else if (abs(i_bus_data_.ch[7] - 2000) < 5)
    {
      i_bus_cmd->sw_d = i_bus_cmd->DOWN;
    }

    i_bus_cmd->vr_a = static_cast<double>((i_bus_data_.ch[8] - 1500.0) / 500.0);
    i_bus_cmd->vr_b = static_cast<double>((i_bus_data_.ch[9] - 1500.0) / 500.0);

    is_update_ = false;
  }
}
}  // namespace rc_ibus