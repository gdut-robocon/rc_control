//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include "rc_hw/hardware_interface/socketcant.h"
#include "rc_hw/hardware_interface/types.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace rc_hw
{
struct CanFrameStamp
{
  can_frame frame;
  ros::Time stamp;
};

class CanBus
{
public:
  /** \brief
   * Initialize device at can_device, retry if fail. Set up header of CAN
   frame.
   *
   * \param bus_name Bus's name(example: can0).
   * \param data_ptr Pointer which point to CAN data.
   */
  CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority);
  /** \brief Read active data from read_buffer_ to data_ptr_, such as
  position,
   * velocity, torque and so on. Clear read_buffer_ after reading.
   *
   * \param time ROS time, but it doesn't be used.
   */
  void read(ros::Time time);
  /** \brief Write commands to can bus.
   *
   */
  void write();

  void write(can_frame* frame);

  const std::string bus_name_;

private:
  /** \brief This function will be called when CAN bus receive message. It
  push
   * frame which received into a vector: read_buffer_.
   *
   * @param frame The frame which socketcan receive.
   */
  void frameCallback(const can_frame& frame);

  can::SocketCAN socket_can_;
  CanDataPtr data_ptr_;
  std::vector<CanFrameStamp> read_buffer_;

  can_frame rm_frame0_{};  // for id 0x201~0x204
  can_frame rm_frame1_{};  // for id 0x205~0x208

  mutable std::mutex mutex_;
};

class DM_Cheetah{
public:
    static void DM_Cheetah_control_cmd(can_frame &frame, uint8_t cmd);
private:
    can::SocketCAN socket_can_;
};

}  // namespace rc_hw
