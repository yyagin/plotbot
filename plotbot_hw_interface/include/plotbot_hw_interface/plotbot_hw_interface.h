/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef PLOTBOT_HW_INTERFACE_PLOTBOT_HW_INTERFACE_H
#define PLOTBOT_HW_INTERFACE_PLOTBOT_HW_INTERFACE_H

#include <signal.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.h"
#include "hardware_interface/hardware_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "joint_limits_interface/joint_limits.h"
#include "joint_limits_interface/joint_limits_interface.h"
#include "joint_limits_interface/joint_limits_rosparam.h"
#include "joint_limits_interface/joint_limits_urdf.h"
#include "plotbot_hw_interface/exponential_moving_average.h"
#include "pluginlib/class_list_macros.hpp"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "urdf_parser/urdf_parser.h"

namespace plotbot_hw_interface
{
static const std::vector<std::string> JOINT_NAMES{ "left_wheel_joint", "right_wheel_joint" };

typedef struct Motor
{
  Motor() : command{ 0.0 }, position{ 0.0 }, velocity{ 0.0 }, effort{ 0.0 }, lpf(0.684)
  {
  }
  ExpMovingAverage<double> lpf;
  std::string joint_name;
  double command;
  double position;
  double velocity;
  double effort;
} Motor;

class PlotbotHardwareInterface : public hardware_interface::RobotHW
{
public:
  PlotbotHardwareInterface();
  PlotbotHardwareInterface(const PlotbotHardwareInterface&) = default;
  PlotbotHardwareInterface& operator=(const PlotbotHardwareInterface&) = default;

  bool init(ros::NodeHandle& /*unused*/, ros::NodeHandle& /*unused*/) override;
  void read(const ros::Time& /*unused*/, const ros::Duration& /*unused*/) override;
  void write(const ros::Time& /*unused*/, const ros::Duration& /*unused*/) override;
  ros::Time getTime() const;
  ros::Duration getPeriod() const;

private:
  void stateCallback(const sensor_msgs::JointState& msg);

  ros::Time pre_time_;
  ros::Time tp_;
  std::vector<Motor> motors_;

  ros::Publisher motor_cmd_pub_;
  ros::Subscriber motor_state_sub_;
  double alpha_;
  bool emergencyBrakeFlag = false;
  bool emergencyButtonFlag = false;
  ros::Time bttn_timestamp = ros::Time::now();
  double max_erpm_;
  double gear_ratio;
  double torque_const;  // physical params.
  double motor_pole_pair;
  double odometry_fix_multiplier = 7.6;
  sensor_msgs::JointState state_msg_;
  sensor_msgs::JointState command_msg_;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_position_interface;
  hardware_interface::VelocityJointInterface joint_velocity_interface;
  hardware_interface::EffortJointInterface joint_effort_interface;

  joint_limits_interface::JointLimits joint_limits;
  joint_limits_interface::PositionJointSaturationInterface limit_position_interface;
  joint_limits_interface::VelocityJointSaturationInterface limit_velocity_interface;
  joint_limits_interface::EffortJointSaturationInterface limit_effort_interface;
};

}  // namespace plotbot_hw_interface

#endif  // PLOTBOT_HW_INTERFACE_PLOTBOT_HW_INTERFACE_H
