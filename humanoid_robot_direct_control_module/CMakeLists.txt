cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_direct_control_module)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    humanoid_robot_controller_msgs
    cmake_modules
    humanoid_robot_framework_common
    humanoid_robot_device
    humanoid_robot_math
    humanoid_robot_kinematics_dynamics
  )
  
  find_package(Boost REQUIRED)
  find_package(Eigen3 REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()


# Specify dependencies
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    humanoid_robot_controller_msgs
    cmake_modules
    humanoid_robot_framework_common
    humanoid_robot_device
    humanoid_robot_math
    humanoid_robot_kinematics_dynamics
    DEPENDS Boost EIGEN3
  )
else()
  ament_package()
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/direct_control_module.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

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

################################################################################
# Test
################################################################################
