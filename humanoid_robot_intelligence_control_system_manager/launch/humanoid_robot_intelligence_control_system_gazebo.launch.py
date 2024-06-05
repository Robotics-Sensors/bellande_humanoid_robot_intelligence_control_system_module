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

import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def ros1_launch_description():
    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_manager", "humanoid_robot_intelligence_control_system_gazebo.launch"]

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Declare launch arguments
    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='true')
    gazebo_robot_name_arg = DeclareLaunchArgument('gazebo_robot_name', default_value='humanoid_robot_intelligence_control_system')
    offset_file_path_arg = DeclareLaunchArgument('offset_file_path', default_value=os.path.join('humanoid_robot_intelligence_control_system_manager', 'config', 'offset.yaml'))
    robot_file_path_arg = DeclareLaunchArgument('robot_file_path', default_value=os.path.join('humanoid_robot_intelligence_control_system_manager', 'config', 'HUMANOID_ROBOT.robot'))
    init_file_path_arg = DeclareLaunchArgument('init_file_path', default_value=os.path.join('humanoid_robot_intelligence_control_system_manager', 'config', 'dxl_init_HUMANOID_ROBOT.yaml'))
    device_name_arg = DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0')

    # Create a list to hold all nodes to be launched
    nodes_to_launch = []

    # ROS2 specific configurations
    ros_launch_arguments = [
        gazebo_arg, gazebo_robot_name_arg, offset_file_path_arg, robot_file_path_arg, init_file_path_arg, device_name_arg
    ]
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_manager',
        executable='humanoid_robot_intelligence_control_system_manager',
        name='humanoid_robot_intelligence_control_system_manager',
        output='screen',
    ))

    # Return the LaunchDescription containing all nodes and arguments
    return LaunchDescription(ros_launch_arguments + nodes_to_launch)


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
