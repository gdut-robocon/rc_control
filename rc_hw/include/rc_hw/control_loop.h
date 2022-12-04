//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include "rc_hw/hardware_interface/hardware_interface.h"

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

namespace rc_hw
{
using namespace std::chrono;
using clock = high_resolution_clock;

class RcRobotHWLoop
{
public:
  /** \brief Create controller manager. Load loop frequency. Start control loop which call @ref
   * rc_hw::RcRobotHWLoop::update() in a frequency.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A pointer which point to hardware_interface.
   */
  RcRobotHWLoop(ros::NodeHandle& nh, std::shared_ptr<RcRobotHW> hardware_interface);

  ~RcRobotHWLoop();
  /** \brief Timed method that reads current hardware's state, runs the controller code once and sends the new commands
   * to the hardware.
   *
   * Timed method that reads current hardware's state, runs the controller code once and sends the new commands to the
   * hardware.
   *
   * Note: we do not use the TimerEvent time difference because it does NOT guarantee that the time source is strictly
   * linearly increasing.
   */

  void update();

private:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Settings
  double cycle_time_error_threshold_{};

  // Timing
  std::thread loop_thread_;
  std::atomic_bool loop_running_;
  double loop_hz_{};
  ros::Duration elapsed_time_;
  clock::time_point last_time_;

  /** ROS Controller Manager and Runner

      This class advertises a ROS interface for loading, unloading, starting, and
      stopping ros_control-based controllers. It also serializes execution of all
      running controllers in \ref update.
  **/
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<RcRobotHW> hardware_interface_;
};
}  // namespace rc_hw