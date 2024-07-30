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
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('mipi_cam'),
        "lib/mipi_cam/config/F37_calibration.yaml")
    print("config_file_path is ", config_file_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            "mipi_config_path", 
            default_value=TextSubstitution(text=str(config_file_path)),
            description='mipi camera calibration file path'),
        DeclareLaunchArgument(
            'mipi_camera_calibration_file_path',
            default_value='default',
            description='mipi camera calibration file path'),
        DeclareLaunchArgument(
            'mipi_out_format',
            default_value='nv12',
            description='mipi camera out format'),
        DeclareLaunchArgument(
            'mipi_image_width',
            default_value='1280',
            description='mipi camera out image width'),
        DeclareLaunchArgument(
            'mipi_image_height',
            default_value='720',
            description='mipi camera out image height'),
        DeclareLaunchArgument(
            'mipi_image_framerate',
            default_value='10.0',
            description='mipi camera out image framerate'),
        DeclareLaunchArgument(
            'mipi_io_method',
            default_value='ros',
            description='mipi camera out io_method'),
        DeclareLaunchArgument(
            'mipi_video_device',
            default_value='default',
            description='mipi camera device'),
        DeclareLaunchArgument(
            'device_mode',
            default_value='dual',
            description='mipi camera device mode single or dual'),
        DeclareLaunchArgument(
            'dual_combine',
            default_value='1',
            description='dual mode output channel'),
        DeclareLaunchArgument(
            'mipi_frame_ts_type',
            default_value='sensor',
            description='type(sensor/realtime) of timestamp for publishing messages'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='default_cam',
            description=''),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='log level'),

        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        # 启动图片发布pkg
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"config_path": LaunchConfiguration('mipi_config_path')},
                {"camera_calibration_file_path": LaunchConfiguration(
                    'mipi_camera_calibration_file_path')},
                {"out_format": LaunchConfiguration('mipi_out_format')},
                {"image_width": LaunchConfiguration('mipi_image_width')},
                {"image_height": LaunchConfiguration('mipi_image_height')},
                {"framerate": LaunchConfiguration('mipi_image_framerate')},
                {"io_method": LaunchConfiguration('mipi_io_method')},
                {"video_device": LaunchConfiguration('mipi_video_device')},
                {"device_mode": LaunchConfiguration('device_mode')},
                {"dual_combine": LaunchConfiguration('dual_combine')},
                {"frame_ts_type": LaunchConfiguration('mipi_frame_ts_type')},
                {"frame_id": LaunchConfiguration('frame_id')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
