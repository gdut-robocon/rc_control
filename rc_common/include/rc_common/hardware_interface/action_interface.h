//
// Created by myx on 2022/11/11.
//

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace rc_control
{
struct ActionData
{
  std::string name;
  double yaw_angle;
  double pitch_angle;
  double roll_angle;
  double pose_x;
  double pose_y;
  double yaw_acc;
};

struct ActionCmd
{
  bool calibration_state;
  bool reset_state;
  bool update_x_state;
  double update_x;
  bool update_y_state;
  double update_y;
  bool update_yaw_state;
  double update_yaw;
};

class ActionHandle
{
public:
  ActionHandle() = default;
  ActionHandle(std::string name, ActionData* action_data, ActionCmd* action_cmd)
    : name_(std::move(name)), action_data_(action_data), action_cmd_(action_cmd)
  {
    if (!action_data_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. action_state pointer is null.");
    if (!action_cmd_)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. action_cmd pointer is null.");
  }

  std::string getName() const
  {
    assert(action_data_);
    return action_data_->name;
  }
  double getYawAngle() const
  {
    assert(action_data_);
    return action_data_->yaw_angle;
  }
  double getPitchAngle() const
  {
    assert(action_data_);
    return action_data_->pitch_angle;
  }
  double getRollAngle() const
  {
    assert(action_data_);
    return action_data_->roll_angle;
  }
  double getPoseX() const
  {
    assert(action_data_);
    return action_data_->pose_x;
  }
  double getPoseY() const
  {
    assert(action_data_);
    return action_data_->pose_y;
  }
  double getYawAcc() const
  {
    assert(action_data_);
    return action_data_->yaw_acc;
  }

  bool getCalibrationState() const
  {
    assert(action_cmd_);
    return action_cmd_->calibration_state;
  }
  bool getResetState() const
  {
    assert(action_cmd_);
    return action_cmd_->reset_state;
  }
  bool getUpdateXState() const
  {
    assert(action_cmd_);
    return action_cmd_->update_x_state;
  }
  bool getUpdateYState() const
  {
    assert(action_cmd_);
    return action_cmd_->update_y_state;
  }
  bool getUpdateYawState() const
  {
    assert(action_cmd_);
    return action_cmd_->update_yaw_state;
  }

  void setCalibration()
  {
    // calibration_state default is true;
    action_cmd_->calibration_state = false;
  }
  void resetAction() const
  {
    action_cmd_->reset_state = true;
  }
  void updatePoseX(double update_x)
  {
    action_cmd_->update_x_state = true;
    action_cmd_->update_x = update_x;
  }
  void updatePoseY(double update_y)
  {
    action_cmd_->update_y_state = true;
    action_cmd_->update_y = update_y;
  }
  void updatePoseYaw(double update_yaw)
  {
    action_cmd_->update_yaw_state = true;
    action_cmd_->update_yaw = update_yaw;
  }

private:
  std::string name_;
  ActionData* action_data_;
  ActionCmd* action_cmd_;
};

class ActionInterface
  : public hardware_interface::HardwareResourceManager<ActionHandle, hardware_interface::ClaimResources>
{
};

}  // namespace rc_control