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
project(humanoid_robot_intelligence_control_system_action_module)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
  )
  
  find_package(Boost REQUIRED)
  
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
    DEPENDS Boost
  )
  
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
  
  add_library(${PROJECT_NAME} src/action_module.cpp)
  
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  
  install(
    DIRECTORY data
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(humanoid_robot_intelligence_control_system_controller_msgs REQUIRED)
  find_package(humanoid_robot_intelligence_control_system_action_module_msgs REQUIRED)
  find_package(humanoid_robot_intelligence_control_system_framework_common REQUIRED)
  find_package(humanoid_robot_intelligence_control_system_device REQUIRED)
  
  find_package(Boost REQUIRED)
  
  include_directories(
    include
    ${Boost_INCLUDE_DIRS}
  )
  
  add_library(${PROJECT_NAME} src/action_module.cpp)
  
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
  )
  
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
  
  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(
    DIRECTORY include/
    DESTINATION include
  )
  
  install(
    DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
  )
  
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(
    rclcpp
    std_msgs
    humanoid_robot_intelligence_control_system_controller_msgs
    humanoid_robot_intelligence_control_system_action_module_msgs
    humanoid_robot_intelligence_control_system_framework_common
    humanoid_robot_intelligence_control_system_device
  )
  
  ament_package()
endif()
