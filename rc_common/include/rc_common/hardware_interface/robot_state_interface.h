//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include <utility>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rc_common/tf_rt_broadcaster.h"

namespace rc_control
{
class RobotStateHandle
{
public:
  RobotStateHandle() = default;
  RobotStateHandle(std::string name, tf2_ros::Buffer* buffer) : name_(std::move(name)), buffer_(buffer)
  {
    if (!buffer)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Tf Buffer data pointer is null.");
  };

  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time)
  {
    return buffer_->lookupTransform(target_frame, source_frame, time);
  }

  bool setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority,
                    bool is_static = false) const
  {
    return buffer_->setTransform(transform, authority, is_static);
  }

  bool setTransform(const std::vector<geometry_msgs::TransformStamped>& transforms, const std::string& authority,
                    bool is_static = false) const
  {
    for (const auto& transform : transforms)
      buffer_->setTransform(transform, authority, is_static);
    return true;
  }

  std::string getName() const
  {
    return name_;
  }

private:
  std::string name_;
  tf2_ros::Buffer* buffer_{};
};

class RobotStateInterface
  : public hardware_interface::HardwareResourceManager<RobotStateHandle, hardware_interface::DontClaimResources>
{
};
}  // namespace rc_control
