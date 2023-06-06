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

#include "plotbot_hw_interface/plotbot_hw_interface.h"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"

namespace plotbot_hw_interface
{
PlotbotHardwareInterface::PlotbotHardwareInterface()
{
}

bool PlotbotHardwareInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  nh.param<double>("low_pass_filter_alpha", alpha_, 0.684);
  Motor left_motor{};
  Motor right_motor{};
  for (auto& joint : JOINT_NAMES)
    {
      Motor motor{};
      motor.joint_name = joint;
      motors_.push_back(motor);
    }
  motor_cmd_pub_ = nh.advertise<sensor_msgs::JointState>("/motor/commands", 1);
  motor_state_sub_ = nh.subscribe("/motor/states", 1, &PlotbotHardwareInterface::stateCallback, this);
  // loads joint limits 

  std::string robot_description_name, robot_description;
  nh.param<std::string>("robot_description_name", robot_description_name, "/robot_description");

  // parses the urdf
  if (nh.getParam(robot_description_name, robot_description))
    {
      const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
      const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(motors_.at(0).joint_name);

      if (getJointLimits(urdf_joint, joint_limits))
        {
          ROS_INFO_STREAM("Joint limits are loaded");
        }
    }

  // reads system parameters
  nh.param<double>("gear_ratio", gear_ratio, 131.25);
  nh.param<double>("odometry_fix_multiplier", odometry_fix_multiplier, 1.0);

  ROS_INFO_STREAM("PARAMETERS: ");
  ROS_INFO_STREAM("- Gear Ratio :" << gear_ratio);
  ROS_INFO_STREAM("- Odometry Fix Multiplier :" << odometry_fix_multiplier);
  {
    size_t i{ 0 };
    for (auto& m : motors_)
      {
        hardware_interface::JointStateHandle state_handle(m.joint_name, &m.position, &m.velocity, &m.effort);
        joint_state_interface.registerHandle(state_handle);
        i++;
      }

    registerInterface(&joint_state_interface);

    ros::Duration(0.5).sleep();
    for (auto& m : motors_)
      {
        
        hardware_interface::JointHandle velocity_handle(joint_state_interface.getHandle(m.joint_name), &m.command);
        hardware_interface::EffortJointInterface effort_handle{};
        joint_velocity_interface.registerHandle(velocity_handle);
      }
    registerInterface(&joint_velocity_interface);
  }
  ROS_INFO_STREAM("Hardware Interface Initialized");
  return true;
}

void PlotbotHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  try
    {
      for (size_t i = 0; i < motors_.size(); ++i)
        {
          if (!state_msg_.name.empty())
            {
              auto& m = motors_.at(i);
              if (m.joint_name == state_msg_.name.at(i))
                {
                  m.velocity = m.lpf(state_msg_.velocity.at(i));
                  m.position = state_msg_.position.at(i);
                }
              else
                {
                  ROS_WARN_STREAM("Motor/Joint names not matching!");
                }
            }
        }
    }
  catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Exception while reading: ");
      ROS_ERROR_STREAM(e.what());
    }
}

void PlotbotHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  limit_velocity_interface.enforceLimits(getPeriod());
  try
    {
      sensor_msgs::JointState cmd;
      for (auto& m : motors_)
        {
          cmd.velocity.push_back((m.command));
          cmd.name.push_back(m.joint_name);
        }
      motor_cmd_pub_.publish(cmd);
    }
  catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Exception while writing: ");
      ROS_ERROR_STREAM(e.what());
    }
}

ros::Time PlotbotHardwareInterface::getTime() const
{
  return ros::Time::now();
}

ros::Duration PlotbotHardwareInterface::getPeriod() const
{
  return ros::Duration(0.02);
}

void PlotbotHardwareInterface::stateCallback(const sensor_msgs::JointState& msg)
{
  state_msg_ = msg;
}

}  // namespace plotbot_hw_interface

PLUGINLIB_EXPORT_CLASS(plotbot_hw_interface::PlotbotHardwareInterface, hardware_interface::RobotHW)
