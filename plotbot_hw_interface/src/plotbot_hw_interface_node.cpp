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

#include <memory>
#include <string>

#include "plotbot_hw_interface/plotbot_hw_interface.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
//   ros::init(argc, argv, "plotbot_hardware_interface_node");
//   ros::NodeHandle nh, nh_private("~");
//   deliverbot_hw_interface::DeliverbotHwInterface deliverbot_hw_interface;
//   deliverbot_hw_interface_rosserial::DeliverbotHwInterface deliverbot_hw_interface_rosserial;
//   std::unique_ptr<controller_manager::ControllerManager> controller_manager;
//   ros::Timer timer;

//   if (findCAN())
//     {
//       deliverbot_hw_interface.init(nh, nh_private);
//       controller_manager = std::make_unique<controller_manager::ControllerManager>(&deliverbot_hw_interface, nh);
//       timer = nh.createTimer(deliverbot_hw_interface.getPeriod(), [&](const ros::TimerEvent& /*event*/) {
//         // sends commands
//         deliverbot_hw_interface.write(deliverbot_hw_interface.getTime(), deliverbot_hw_interface.getPeriod());

//         // updates the hardware interface control
//         controller_manager->update(deliverbot_hw_interface.getTime(), deliverbot_hw_interface.getPeriod());

//         // gets current states
//         deliverbot_hw_interface.read(deliverbot_hw_interface.getTime(), deliverbot_hw_interface.getPeriod());
//       });
//     }
//   else
//     {
//       node_logger.critical("CAN interface can not found!");
//     }

//   ros::AsyncSpinner spinner(2);

//   spinner.start();
//   ros::waitForShutdown();
//   // spinner.stop();

  return EXIT_SUCCESS;
}
