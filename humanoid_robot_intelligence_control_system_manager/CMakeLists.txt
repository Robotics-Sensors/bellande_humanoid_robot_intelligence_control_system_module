# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_walking_module_msgs
    humanoid_robot_intelligence_control_system_online_walking_module_msgs
    humanoid_robot_intelligence_control_system_tuning_module_msgs
    cmake_modules
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_controller
    humanoid_robot_intelligence_control_system_math
    humanoid_robot_intelligence_control_system_kinematics_dynamics
    humanoid_robot_intelligence_control_system_balance_control
    humanoid_robot_intelligence_control_system_open_cr_module
    humanoid_robot_intelligence_control_system_base_module
    humanoid_robot_intelligence_control_system_head_control_module
    humanoid_robot_intelligence_control_system_walking_module
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_direct_control_module
    humanoid_robot_intelligence_control_system_online_walking_module
    humanoid_robot_intelligence_control_system_tuning_module
  )
else()
  find_package(ament_cmake REQUIRED COMPONENTS
    rclcpp
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_walking_module_msgs
    humanoid_robot_intelligence_control_system_online_walking_module_msgs
    humanoid_robot_intelligence_control_system_tuning_module_msgs
    cmake_modules
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_controller
    humanoid_robot_intelligence_control_system_math
    humanoid_robot_intelligence_control_system_kinematics_dynamics
    humanoid_robot_intelligence_control_system_balance_control
    humanoid_robot_intelligence_control_system_open_cr_module
    humanoid_robot_intelligence_control_system_base_module
    humanoid_robot_intelligence_control_system_head_control_module
    humanoid_robot_intelligence_control_system_walking_module
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_direct_control_module
    humanoid_robot_intelligence_control_system_online_walking_module
    humanoid_robot_intelligence_control_system_tuning_module
  )
endif()


# Specify dependencies
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_walking_module_msgs
    humanoid_robot_intelligence_control_system_online_walking_module_msgs
    cmake_modules
    dynamixel_sdk
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
    humanoid_robot_intelligence_control_system_controller
    humanoid_robot_intelligence_control_system_math
    humanoid_robot_intelligence_control_system_kinematics_dynamics
    humanoid_robot_intelligence_control_system_balance_control
    humanoid_robot_intelligence_control_system_open_cr_module
    humanoid_robot_intelligence_control_system_base_module
    humanoid_robot_intelligence_control_system_head_control_module
    humanoid_robot_intelligence_control_system_walking_module
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_direct_control_module
    humanoid_robot_intelligence_control_system_online_walking_module
    humanoid_robot_intelligence_control_system_tuning_module
    humanoid_robot_intelligence_control_system_tuning_module_msgs
  )
endif()


