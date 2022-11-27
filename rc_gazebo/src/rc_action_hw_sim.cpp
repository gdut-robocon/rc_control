//
// Created by myx on 2022/11/17.
//
#include "rc_gazebo/rc_action_hw_sim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace rc_gazebo
{
bool RcRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                           gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                           std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&rc_action_sim_interface_);
  XmlRpc::XmlRpcValue xml_rpc_value;

  if (!model_nh.getParam("actions", xml_rpc_value))
    ROS_WARN("No action specified");
  else
    parseAction(xml_rpc_value, parent_model);
  world_ = parent_model->GetWorld();  // For gravity
  return ret;
}

void RcRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  for (auto& action : action_sim_datas_)
  {
    std::mt19937 gen(time.toNSec());
    ignition::math::Pose3d pose = action.link_ptr->WorldPose();
    action.time_stamp = time;

    //    std::uniform_real_distribution<double> dist_angle_error(0.0, angle_error_ / period.toSec());
    //    cumulative_angle_error_ += dist_angle_error(gen);
    //
    //    std::uniform_real_distribution<double> dist_poseX_error(0.0, pose_error_ / (pose.X() - last_poseX_));
    //    cumulative_poseX_error_ += dist_poseX_error(gen);
    //
    //    std::uniform_real_distribution<double> dist_poseY_error(0.0, pose_error_ / (pose.Y() - last_poseY_));
    //    cumulative_poseY_error_ += dist_poseY_error(gen);

    action.action_data.yaw_angle = pose.Yaw() + cumulative_angle_error_;
    action.action_data.pitch_angle = pose.Pitch() + cumulative_angle_error_;
    action.action_data.roll_angle = pose.Roll() + cumulative_angle_error_;
    action.action_data.pose_x = pose.X() + cumulative_poseX_error_;
    action.action_data.pose_y = pose.Y() + cumulative_poseY_error_;
    action.action_data.yaw_acc = (pose.Yaw() - last_yaw_angle_) / period.toSec();

    //    ROS_INFO_STREAM("action.action_data.yaw_angle: " << action.action_data.yaw_angle);
    //    ROS_INFO_STREAM("action.action_data.pitch_angle: " << action.action_data.pitch_angle);
    //    ROS_INFO_STREAM("action.action_data.roll_angle: " << action.action_data.roll_angle);
    //    ROS_INFO_STREAM("action.action_data.pose_x: " << action.action_data.pose_x);
    //    ROS_INFO_STREAM("action.action_data.pose_y: " << action.action_data.pose_y);
    //    ROS_INFO_STREAM("action.action_data.yaw_acc: " << action.action_data.yaw_acc);

    last_yaw_angle_ = pose.Yaw();
    last_poseX_ = pose.X();
    last_poseY_ = pose.Y();
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_)
    cmd = 0;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0;
}

void RcRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
  for (auto& action : action_sim_datas_)
  {
    ignition::math::Pose3d pose = action.link_ptr->WorldPose();
    action.time_stamp = time;
    // calibration_state default id True.
    if (!action.action_cmd.calibration_state)
    {
      ROS_INFO_STREAM("Received calibration_cmd of action: " << action.action_data.name);
      action.action_cmd.calibration_state = true;
    }

    if (action.action_cmd.reset_state)
    {
      ROS_INFO_STREAM("Received reset_cmd of action: " << action.action_data.name);
      action.action_data.yaw_angle = 0.0;
      action.action_data.pitch_angle = 0.0;
      action.action_data.roll_angle = 0.0;
      action.action_data.pose_x = 0.0;
      action.action_data.pose_y = 0.0;
      action.action_data.yaw_acc = 0.0;

      action.action_cmd.reset_state = false;
    }

    if (action.action_cmd.update_yaw_state)
    {
      ROS_INFO_STREAM("Received update_yaw_cmd of action: " << action.action_data.name);
      action.action_data.yaw_angle = action.action_cmd.update_yaw;
      action.action_cmd.update_yaw_state = false;
    }

    if (action.action_cmd.update_x_state)
    {
      ROS_INFO_STREAM("Received update_x__cmd of action: " << action.action_data.name);
      action.action_data.pose_x = action.action_cmd.update_x;
      action.action_cmd.update_x_state = false;
    }

    if (action.action_cmd.update_y_state)
    {
      ROS_INFO_STREAM("Received update_y_cmd of action: " << action.action_data.name);
      action.action_data.pose_y = action.action_cmd.update_y;
      action.action_cmd.update_y_state = false;
    }
  }
}

void RcRobotHWSim::parseAction(XmlRpc::XmlRpcValue& action_sim_datas, const gazebo::physics::ModelPtr& parent_model)
{
  ROS_ASSERT(action_sim_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = action_sim_datas.begin(); it != action_sim_datas.end(); ++it)
  {
    if (!it->second.hasMember("frame_id"))
    {
      ROS_ERROR_STREAM("Action " << it->first << " has no associated frame id.");
      continue;
    }
    else if (!it->second.hasMember("angle_error"))
    {
      ROS_ERROR_STREAM("Action " << it->first << " has no associated angle_error.");
      continue;
    }
    else if (!it->second.hasMember("pose_error"))
    {
      ROS_ERROR_STREAM("Action " << it->first << " has no associated pose_error.");
      continue;
    }
    XmlRpc::XmlRpcValue angle_error = action_sim_datas[it->first]["angle_error"];
    ROS_ASSERT(angle_error.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    angle_error_ = angle_error;
    XmlRpc::XmlRpcValue pose_error = action_sim_datas[it->first]["pose_error"];
    ROS_ASSERT(pose_error.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    pose_error_ = pose_error;

    cumulative_angle_error_ = 0.0;
    cumulative_poseX_error_ = 0.0;
    cumulative_poseY_error_ = 0.0;
    last_yaw_angle_ = 0.0;
    last_poseX_ = 0.0;
    last_poseY_ = 0.0;

    std::string frame_id = action_sim_datas[it->first]["frame_id"];
    gazebo::physics::LinkPtr link_ptr = parent_model->GetLink(frame_id);
    if (link_ptr == nullptr)
    {
      ROS_WARN("Action %s is not specified in urdf.", it->first.c_str());
      continue;
    }

    rc_control::ActionData action_data;
    action_data = { .name = it->first,
                    .yaw_angle = 0.0,
                    .pitch_angle = 0.0,
                    .roll_angle = 0.0,
                    .pose_x = 0.0,
                    .pose_y = 0.0,
                    .yaw_acc = 0.0 };
    rc_control::ActionCmd action_cmd;
    action_cmd = { .calibration_state = true,
                   .reset_state = false,
                   .update_x_state = false,
                   .update_x = 0.0,
                   .update_y_state = false,
                   .update_y = 0.0,
                   .update_yaw_state = false,
                   .update_yaw = 0.0 };

    ActionSimData action_data_sim = {
      .link_ptr = link_ptr, .time_stamp = {}, .action_data = action_data, .action_cmd = action_cmd
    };

    action_sim_datas_.push_back(action_data_sim);

    rc_control::ActionHandle action_sim_handle(it->first, &action_sim_datas_.back().action_data,
                                               &action_sim_datas_.back().action_cmd);
    rc_action_sim_interface_.registerHandle(action_sim_handle);
  }
}

}  // namespace rc_gazebo

PLUGINLIB_EXPORT_CLASS(rc_gazebo::RcRobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
