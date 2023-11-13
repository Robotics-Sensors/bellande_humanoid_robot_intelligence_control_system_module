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
    open_cr_module
    humanoid_robot_intelligence_control_system_base_module
    humanoid_robot_intelligence_control_system_head_control_module
    humanoid_robot_intelligence_control_system_walking_module
    humanoid_robot_intelligence_control_system_action_module
    humanoid_robot_intelligence_control_system_direct_control_module
    humanoid_robot_intelligence_control_system_online_walking_module
    humanoid_robot_intelligence_control_system_tuning_module
  )
else()
  find_package(ament_cmake REQUIRED)
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
    open_cr_module
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


################################################################################
# Build
################################################################################
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/humanoid_robot_intelligence_control_system_manager.cpp)
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
