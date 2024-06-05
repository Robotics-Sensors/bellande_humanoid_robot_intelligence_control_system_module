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

import sys
import subprocess
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_manager", "humanoid_robot_intelligence_control_system_manager.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)

def ros2_launch_description():
    # Declare launch arguments
    offset_file_path_arg = DeclareLaunchArgument('offset_file_path')
    robot_file_path_arg = DeclareLaunchArgument('robot_file_path')
    init_file_path_arg = DeclareLaunchArgument('init_file_path')
    device_name_arg = DeclareLaunchArgument('device_name')

    # Create a list to hold all nodes to be launched
    nodes_to_launch = []

    # ROS2 specific configurations
    ros_launch_arguments = [
        offset_file_path_arg, robot_file_path_arg, init_file_path_arg, device_name_arg
    ]
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_manager',
        executable='humanoid_robot_intelligence_control_system_manager',
        name='humanoid_robot_intelligence_control_system_manager',
        output='screen',
        parameters=[
            {'gazebo': False},
            {'gazebo_robot_name': 'humanoid_robot_intelligence_control_system'},
            {'offset_file_path': LaunchConfiguration('offset_file_path')},
            {'robot_file_path': LaunchConfiguration('robot_file_path')},
            {'init_file_path': LaunchConfiguration('init_file_path')},
            {'device_name': LaunchConfiguration('device_name')},
            {'angle_unit': 30}
        ],
    ))

    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_localization',
        executable='humanoid_robot_intelligence_control_system_localization',
        name='humanoid_robot_intelligence_control_system_localization',
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

