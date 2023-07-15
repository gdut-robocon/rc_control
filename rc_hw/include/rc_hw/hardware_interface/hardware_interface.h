//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <rc_common/hardware_interface/action_interface.h>
#include <rc_common/hardware_interface/actuator_extra_interface.h>
#include <rc_common/hardware_interface/gpio_interface.h>
#include <rc_common/hardware_interface/rc_imu_sensor_interface.h>
#include <rc_common/hardware_interface/robot_state_interface.h>
#include <rc_msgs/ActuatorState.h>

#include "can_bus.h"
#include "gpio_manager.h"
#include "action_manager.h"
#include "DT35_laser.h"

namespace rc_hw
{
class RcRobotHW : public hardware_interface::RobotHW
{
public:
  RcRobotHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load
   * urdf of robot. Set up transmission and joint limit. Get configuration of
   * can bus and create data pointer which point to data received from Can
   bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Comunicate with hardware. Get datas, status of robot.
   *
   * Call @ref rc_hw::CanBus::read(). Check whether temperature of actuator is
   * too high and whether actuator is offline. Propagate actuator state to
   joint
   * state for the stored transmission. Set all cmd to zero to avoid crazy
   soft
   * limit oscillation when not controller loaded(all controllers update after
   * read()).
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref
   * rc_hw::CanBus::write(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  void setCanBusThreadPriority(int thread_priority);

private:
  /** \brief Check whether some coefficients that are related to actuator are
   * set up and load these coefficients.
   *
   * Check whether some coefficients that are related to actuator are set up
   and
   * load these coefficients.
   *
   * @param act_coeffs Coefficients you want to check and load.
   * @return True if all coefficients are set up.
   */
  bool parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs);

  /** \brief Check whether actuator is specified and load specified params.
   *
   * Check whether actuator is specified and load specified params.
   *
   * @param act_datas Params you want to check and load.
   * @param robot_hw_nh Root node-handle of a ROS node.
   * @return True if all params are set up.
   */
  bool parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh);

  /** \brief Check whether some params that are related to imu are set up and
   * load these params.
   *
   * Check whether some params that are related to imu are set up and load
   these
   * params.
   *
   * @param imu_datas Params you want to check
   * @param robot_hw_nh Root node-handle of a ROS node
   * @return True if all params are set up.
   */
  bool parseImuData(XmlRpc::XmlRpcValue& imu_datas, ros::NodeHandle& robot_hw_nh);

  /** \brief Check whether somme params that are related to gpio are set up and
   * load these params.
   *
   * Check whether somme params that are related to gpio are set up and load
   * these params.
   *
   * @param gpio_datas Params you want to check and load
   * @param robot_hw_nh A handle of a ROS node
   * @return True if all params are set up.
   */
  bool parseGpioData(XmlRpc::XmlRpcValue& gpio_datas, ros::NodeHandle& robot_hw_nh);

  /** \brief Check whether somme params that are related to action are set up and
   * load these params.
   *
   * Check whether somme params that are related to action are set up and load
   * these params.
   *
   * @param action_datas Params you want to check and load
   * @param robot_hw_nh A handle of a ROS node
   * @return True if all params are set up.
   */
  bool parseActionData(XmlRpc::XmlRpcValue& action_datas, ros::NodeHandle& robot_hw_nh);

  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param root_nh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& root_nh);

  bool setupTransmission(ros::NodeHandle& root_nh);
  /** \brief Set up joint limit.
   *
   * Set up joint limit.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @return True if successful.
   */
  bool setupJointLimit(ros::NodeHandle& root_nh);
  /** \brief Publish actuator's state to a topic named "/actuator_states".
   *
   * Publish actuator's state to a topic named "/actuator_states".
   *
   * @param time Current time
   */
  void publishActuatorState(const ros::Time& time);

  bool is_actuator_specified_ = false;
  int thread_priority_;
  // Interface
  std::vector<CanBus*> can_buses_{};
  GpioManager gpio_manager_{};
  ActionManager action_manager_{};
  DT35_laser dt35_laser_{};

  rc_control::ActionInterface action_interface_;
  rc_control::GpioStateInterface gpio_state_interface_;
  rc_control::GpioCommandInterface gpio_command_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  rc_control::ActuatorExtraInterface act_extra_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  rc_control::RobotStateInterface robot_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  rc_control::RcImuSensorInterface rc_imu_sensor_interface_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface* jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  // Actuator
  std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // Imu
  std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};

  ros::Time last_publish_time_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rc_msgs::ActuatorState>> actuator_state_pub_;
};

}  // namespace rc_hw
