################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(humanoid_robot_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  humanoid_robot_controller_msgs
  humanoid_robot_action_module_msgs
  humanoid_robot_walking_module_msgs
  humanoid_robot_online_walking_module_msgs
  humanoid_robot_tuning_module_msgs
  cmake_modules
  dynamixel_sdk
  humanoid_robot_framework_common
  humanoid_robot_device
  humanoid_robot_controller
  humanoid_robot_math
  humanoid_robot_kinematics_dynamics
  humanoid_robot_balance_control
  open_cr_module
  humanoid_robot_base_module
  humanoid_robot_head_control_module
  humanoid_robot_walking_module
  humanoid_robot_action_module
  humanoid_robot_direct_control_module
  humanoid_robot_online_walking_module
  humanoid_robot_tuning_module
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  humanoid_robot_controller_msgs
  humanoid_robot_action_module_msgs
  humanoid_robot_walking_module_msgs
  humanoid_robot_online_walking_module_msgs
  cmake_modules
  dynamixel_sdk
  humanoid_robot_framework_common
  humanoid_robot_device
  humanoid_robot_controller
  humanoid_robot_math
  humanoid_robot_kinematics_dynamics
  humanoid_robot_balance_control
  open_cr_module
  humanoid_robot_base_module
  humanoid_robot_head_control_module
  humanoid_robot_walking_module
  humanoid_robot_action_module
  humanoid_robot_direct_control_module
  humanoid_robot_online_walking_module
  humanoid_robot_tuning_module
  humanoid_robot_tuning_module_msgs
)

################################################################################
# Build
################################################################################
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/humanoid_robot_manager.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
