//
// Created by myx on 2022/11/11.
//
// ref:https://github.com/rm-controls

#pragma once

#include <rc_common/filters/imu_filter_base.h>
#include <rc_common/filters/lp_filter.h>
#include <string>
#include <unordered_map>

namespace rc_hw
{
struct ActCoeff
{
  double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
      act2effort_offset, kp2act,
      kd2act;  // for MIT Cheetah motor
};

struct ActData
{
  std::string name;
  std::string type;
  ros::Time stamp;
  uint64_t seq;
  bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
  uint16_t q_raw;
  int16_t qd_raw;
  uint8_t temp;
  int64_t q_circle;
  uint16_t q_last;
  double frequency;
  double pos, vel, effort;
  double cmd_pos, cmd_vel, cmd_effort, exe_effort;
  double offset;
  // For multiple cycle under absolute encoder (RoboMaster motor)
  LowPassFilter* lp_filter;
};

struct ImuData
{
  ros::Time time_stamp;
  std::string imu_name;
  double ori[4];
  double angular_vel[3], linear_acc[3];
  double angular_vel_offset[3];
  double ori_cov[9], angular_vel_cov[9], linear_acc_cov[9];
  double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
  bool accel_updated, gyro_updated, camera_trigger;
  bool enabled_trigger;
  rc_common::ImuFilterBase* imu_filter;
};

struct TofData
{
  double strength;
  double distance;
};

struct CanDataPtr
{
  std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
  std::unordered_map<int, ActData>* id2act_data_;
  std::unordered_map<int, ImuData>* id2imu_data_;
  std::unordered_map<int, TofData>* id2tof_data_;
};

struct ActionData
{
  ros::Time time_stamp;
  double yaw_angle, pitch_angle, roll_angle, pose_x, pose_y, yaw_acc;
};
}  // namespace rc_hw
