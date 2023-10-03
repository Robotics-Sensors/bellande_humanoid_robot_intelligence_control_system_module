cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_action_module)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    humanoid_robot_controller_msgs
    humanoid_robot_action_module_msgs
    humanoid_robot_framework_common
    humanoid_robot_device
  )
else()
  find_package(ament_cmake REQUIRED)
endif()

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################

find_package(Boost REQUIRED)

# Specify dependencies
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    humanoid_robot_controller_msgs
    humanoid_robot_action_module_msgs
    humanoid_robot_framework_common
    humanoid_robot_device
    DEPENDS Boost
  )
else()
  ament_package()
endif()


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/action_module.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
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

################################################################################
# Test
################################################################################
