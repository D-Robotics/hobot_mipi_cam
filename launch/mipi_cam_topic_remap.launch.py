# Copyright (c) 2024，D-Robotics.
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
        "lib/mipi_cam/config/")
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
            default_value='960',
            description='mipi camera out image width'),
        DeclareLaunchArgument(
            'mipi_image_height',
            default_value='544',
            description='mipi camera out image height'),
        DeclareLaunchArgument(
            'mipi_image_framerate',
            default_value='30.0',
            description='mipi camera out image framerate'),
        DeclareLaunchArgument(
            'mipi_channel',
            default_value='0',
            description='mipi camera host channel'),
        DeclareLaunchArgument(
            'mipi_io_method',
            default_value='shared_mem',
            description='mipi camera out io_method'),
        DeclareLaunchArgument(
            'mipi_video_device',
            default_value='default',
            description='mipi camera device'),
        DeclareLaunchArgument(
            'mipi_frame_ts_type',
            default_value='sensor',
            description='type(sensor/realtime) of timestamp for publishing messages'),
        DeclareLaunchArgument(
            'mipi_rotation',
            default_value='0.0',
            description='mipi camera out image rotation'),
        DeclareLaunchArgument(
            'mipi_cal_rotation',
            default_value='0.0',
            description='mipi camera calibration rotation'),
        DeclareLaunchArgument(
            'mipi_gdc_bin_file',
            default_value='default',
            description='mipi camera gdc file path'),
        DeclareLaunchArgument(
            'mipi_lpwm_enable',
            default_value='False',
            description='mipi dual camera lpwm enable'),
        DeclareLaunchArgument(
            'mipi_ros_topic',
            default_value='/image_raw_alias',
            description='mipi ros topic alias'),
        DeclareLaunchArgument(
            'mipi_shared_mem_topic',
            default_value='/hbmem_img_alias',
            description='mipi shared_mem topic alias'),
        DeclareLaunchArgument(
            'mipi_camera_info_topic',
            default_value='/camera_info_alias',
            description='mipi camera_info topic alias'),
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
                {"framerate": LaunchConfiguration('mipi_image_framerate')},
                {"image_width": LaunchConfiguration('mipi_image_width')},
                {"image_height": LaunchConfiguration('mipi_image_height')},
                {"io_method": LaunchConfiguration('mipi_io_method')},
                {"channel": LaunchConfiguration('mipi_channel')},
                {"video_device": LaunchConfiguration('mipi_video_device')},
                {"frame_ts_type": LaunchConfiguration('mipi_frame_ts_type')},
                {"rotation": LaunchConfiguration('mipi_rotation')},
                {"lpwm_enable": LaunchConfiguration('mipi_lpwm_enable')},
                {"cal_rotation": LaunchConfiguration('mipi_cal_rotation')},
                {"gdc_bin_file": LaunchConfiguration('mipi_gdc_bin_file')},
            ],
            remappings=[
                ("/image_raw", LaunchConfiguration('mipi_ros_topic')),
                ("/camera_info", LaunchConfiguration('mipi_camera_info_topic')),
                ("/hbmem_img", LaunchConfiguration('mipi_shared_mem_topic'))],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])
