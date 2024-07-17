# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cam_package_name = 'hobot_stereo_mipi_cam'
    imu_package_name = 'imx219_stereo_imu'

    cam_config_path = os.path.join(get_package_share_directory(cam_package_name), 'config', 'stereo_cam_config.yaml')
    imu_config_path = os.path.join(get_package_share_directory(imu_package_name), 'config', 'stereo_imu_config.yaml')
    ld = LaunchDescription()

    cam_node = Node(
        package='hobot_stereo_mipi_cam',
        executable='hobot_stereo_mipi_cam',
        output='screen',
        parameters=[cam_config_path],
        arguments=['--ros-args', '--log-level', 'info']
    )

    imu_node = Node(
        package='imx219_stereo_imu',
        executable='imx219_stereo_imu_node',
        output='screen',
        parameters=[imu_config_path],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(cam_node)
    ld.add_action(imu_node)

    return ld

