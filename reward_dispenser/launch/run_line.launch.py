"""

Relevant links:

* https://docs.ros.org/en/galactic/Tutorials/Launch-system.html
* https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Creating-Launch-Files.html
* https://docs.ros.org/en/galactic/Guides/Launch-file-different-formats.html
* https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst
"""

# Copyright 2023 Jonathan Shulgach
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # Set the ROS2 domain space
    ros_domain = ExecuteProcess( cmd=["set ROS_DOMAIN_ID=42"], shell=True, output="screen" )


    # Reward dispenser node
    reward_node = Node(package='reward_dispenser',
                       namespace='reward',
                       executable='run_line',
                       parameters=[{'dispense_time':60}]
                       )

    
    return LaunchDescription([ros_domain, 
                              reward_node,
                              ])