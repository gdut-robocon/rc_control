//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include <hardware_interface/imu_sensor_interface.h>

namespace rc_control
{
class RcImuSensorHandle : public hardware_interface::ImuSensorHandle
{
public:
  RcImuSensorHandle() = default;

  RcImuSensorHandle(const hardware_interface::ImuSensorHandle& imu_sensor_handle, ros::Time* time_stamp)
    : ImuSensorHandle(imu_sensor_handle), time_stamp_(time_stamp)
  {
    if (!time_stamp_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + imu_sensor_handle.getName() +
                                                           "'. Time stamp pointer is null");
    }
  }
  ros::Time getTimeStamp()
  {
    assert(time_stamp_);
    return *time_stamp_;
  }

private:
  ros::Time* time_stamp_ = { nullptr };
};

class RcImuSensorInterface
  : public hardware_interface::HardwareResourceManager<RcImuSensorHandle, hardware_interface::DontClaimResources>
{
};
}  // namespace rc_control
