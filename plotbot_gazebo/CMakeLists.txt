cmake_minimum_required(VERSION 2.8.3)

project(plotbot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
