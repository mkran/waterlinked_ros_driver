cmake_minimum_required(VERSION 2.8.3)
project(waterlinked_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/WaterlinkedParams.cfg
)

catkin_package()
