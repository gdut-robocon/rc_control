//
// Created by myx on 2022/11/17.
//

#pragma once

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <rc_common/hardware_interface/action_interface.h>

namespace rc_gazebo
{
struct ActionSimData
{
  gazebo::physics::LinkPtr link_ptr;
  ros::Time time_stamp;
  rc_control::ActionData action_data;
  rc_control::ActionCmd action_cmd;
};

class RcRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;

  void writeSim(ros::Time time, ros::Duration period) override;

private:
  void parseAction(XmlRpc::XmlRpcValue& action_sim_datas, const gazebo::physics::ModelPtr& parent_model);

  rc_control::ActionInterface rc_action_sim_interface_;
  gazebo::physics::WorldPtr world_;
  std::vector<ActionSimData> action_sim_datas_;

  double angle_error_;
  double cumulative_angle_error_;

  double pose_error_;
  double cumulative_poseX_error_;
  double cumulative_poseY_error_;
  double last_yaw_angle_;
  double last_poseX_;
  double last_poseY_;
};

}  // namespace rc_gazebo
