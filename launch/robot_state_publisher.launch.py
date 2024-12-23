#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """
    Generate the launch description for the robot_state_publisher node.

    This function retrieves the URDF file for the robot, processes it using xacro,
    and then configures and launches the robot_state_publisher node with the
    processed URDF as a parameter.
    """
    # Get the URDF file path
    pkg_name = 'digitwin'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'digitwin.urdf.xacro')
    
    # Process the URDF file using xacro
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # Configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # Return the LaunchDescription containing the node
    return LaunchDescription([robot_state_publisher_node])

if __name__ == '__main__':
    generate_launch_description()

# ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ammar/dev_ws/src/digitwin/urdf/digitwin.urdf.xacro)"