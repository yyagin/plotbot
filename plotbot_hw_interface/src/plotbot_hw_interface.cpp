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
  std::map<std::string, int> motors{};
  nh.param<double>("low_pass_filter_alpha", alpha_, 0.684);

  motor_cmd_pub_ = nh.advertise<sensor_msgs::JointState>("/motor/commands", 1);
  motor_state_sub_ = nh.subscribe("/motor/states", 1, &PlotbotHardwareInterface::stateCallback, this);
  // loads joint limits

  std::string robot_description_name, robot_description;
  nh.param<std::string>("robot_description_name", robot_description_name, "/robot_description");

  // parses the urdf
  if (nh.getParam(robot_description_name, robot_description))
    {
      const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
      const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(motors.at(0).joint_name);

      if (getJointLimits(urdf_joint, joint_limits))
        {
          hw_interface_logger_.info("Joint limits are loaded");
        }
    }

  // reads system parameters
  nh.param<double>("gear_ratio", gear_ratio, 1.0);
  nh.param<double>("torque_const", torque_const, 1.0);
  nh.param<double>("motor_pole_pair", motor_pole_pair, 1.0);
  nh.param<double>("odometry_fix_multiplier", odometry_fix_multiplier, 1.0);

  ROS_INFO_STREAM("PARAMETERS: ");
  ROS_INFO_STREAM("- Gear Ratio :" << gear_ratio);
  ROS_INFO_STREAM("- Motor Pole Pair :" << motor_pole_pair);
  ROS_INFO_STREAM("- Odometry Fix Multiplier :" << odometry_fix_multiplier);
  {
    size_t i{ 0 };
    for (auto& m : vesc_motors)
      {
        hardware_interface::JointStateHandle state_handle(m.joint_name, &m.position, &m.velocity, &m.effort);
        joint_state_interface.registerHandle(state_handle);
        i++;
      }

    registerInterface(&joint_state_interface);
    ros::Duration(1).sleep();
    for (auto& m : vesc_motors)
      {
        hardware_interface::JointHandle velocity_handle(joint_state_interface.getHandle(m.joint_name), &m.command);
        joint_velocity_interface.registerHandle(velocity_handle);
      }
    // registers a state handle and its interface
    registerInterface(&joint_velocity_interface);
  }
  hw_interface_logger_.info("Hardware Interface Initialized");
  return true;
}

void PlotbotHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  auto dt = static_cast<double>((getTime() - pre_time_).nsec) / 1e9;
  pre_time_ = getTime();
  try
    {
      size_t i{ 0 };
      for (auto& m : vesc_motors)
        {
          auto erpm = m.ptr->getRPM();
          if (erpm > max_erpm_)
            erpm = max_erpm_;
          if (erpm < -1 * max_erpm_)
            erpm = -1 * max_erpm_;

          m.velocity = (m.lpf(erpm) / motor_pole_pair) / 60.0 * 2.0 * M_PI / gear_ratio;
          m.position += m.velocity * dt;
          i++;
        }
    }
  catch (const std::exception& e)
    {
      hw_interface_logger_.error("Exception while reading: ");
      hw_interface_logger_.error(e.what());
    }
}

void PlotbotHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  limit_velocity_interface.enforceLimits(getPeriod());
  // converts the velocity unit: rad/s or m/s -> rpm
  // Motor pole pair added to calculate ERPM, VESC's speed controller input
  // must be "ERPM"
  try
    {
      hardware_msgs::MotorCmd cmd;
      double ref_velocity_rpm{};
      size_t i{};
      for (auto& m : vesc_motors)
        {
          // ROS_INFO_STREAM(" ID First" << id.first <<"  VESC IDs: " << id.second);
          ref_velocity_rpm = m.command * 60.0 / 2.0 / M_PI * gear_ratio;
          if (!emergencyBrakeFlag)
            {
              m.ptr->setRPM(ref_velocity_rpm * motor_pole_pair);
            }
          else
            {
              m.ptr->setDuty(0);
            }
        }
      cmd.motor1_cmd = ref_velocity_rpm * motor_pole_pair;
      cmd.motor2_cmd = ref_velocity_rpm * motor_pole_pair;
      cmd.motor3_cmd = ref_velocity_rpm * motor_pole_pair;
      cmd.motor4_cmd = ref_velocity_rpm * motor_pole_pair;
      cmd.emergency_brake = emergencyBrakeFlag;
      motor_cmd_pub_.publish(cmd);
      std_msgs::Int8 emergency_msg{};
      emergency_msg.data = static_cast<int8_t>(emergencyBrakeFlag);
      emergency_brake_pub_.publish(emergency_msg);
    }
  catch (const std::exception& e)
    {
      hw_interface_logger_.error("Exception while writing: ");
      hw_interface_logger_.error(e.what());
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

bool PlotbotHardwareInterface::motorEmergencyCallback(hardware_msgs::EmergencyBrake::Request& req,
                                                      hardware_msgs::EmergencyBrake::Response& res)
{
  emergencyBrakeFlag = req.request;
  return true;
}

void PlotbotHardwareInterface::bttnEmergencyCallBack(const sensor_msgs::Joy::ConstPtr& data)
{
  if (data->buttons[1] == 1 && !emergencyButtonFlag)
    {
      emergencyBrakeFlag = !emergencyBrakeFlag;
      emergencyButtonFlag = true;
    }
  else if (data->buttons[1] == 0 && emergencyButtonFlag)
    {
      emergencyButtonFlag = false;
    }
}

}  // namespace plotbot_hw_interface

PLUGINLIB_EXPORT_CLASS(plotbot_hw_interface::PlotbotHardwareInterface, hardware_interface::RobotHW)
