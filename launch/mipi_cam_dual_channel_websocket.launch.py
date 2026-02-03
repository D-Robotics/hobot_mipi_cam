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
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)

    mipi_lpwm_enable_arg = DeclareLaunchArgument(
        'mipi_lpwm_enable',
        default_value='False',
        description='mipi lpwm enable')
    
    mipi_camera_calibration_file_path_arg = DeclareLaunchArgument(
        'mipi_camera_calibration_file_path',
        default_value='default',
        description='mipi lpwm enable')
    
    mipi_image_width_arg = DeclareLaunchArgument(
        'mipi_image_width',
        default_value='1280',
        description='mipi width')

    mipi_image_height_arg = DeclareLaunchArgument(
        'mipi_image_height',
        default_value='640',
        description='mipi height')

    mipi_rotation_arg = DeclareLaunchArgument(
        'mipi_rotation',
        default_value='0.0',
        description='mipi camera out image rotation')

    mipi_cal_rotation_arg = DeclareLaunchArgument(
        'mipi_cal_rotation',
        default_value='0.0',
        description='mipi camera calibration rotation')

    mipi_gdc_enable_arg = DeclareLaunchArgument(
        'mipi_gdc_enable',
        default_value='True',
        description='mipi gdc enable')
    mipi_image_framerate_arg = DeclareLaunchArgument(
        'mipi_image_framerate',
        default_value='10.0',
        description='mipi camera out image framerate')
    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch/mipi_cam_dual_channel.launch.py')),
        launch_arguments={
            'mipi_image_width': LaunchConfiguration('mipi_image_width'),
            'mipi_image_height': LaunchConfiguration('mipi_image_height'),
            'mipi_image_framerate': LaunchConfiguration('mipi_image_framerate'),
            'mipi_io_method': 'ros',
            'device_mode': 'dual',
            'dual_combine': '2',
            'mipi_channel': '2',
            'mipi_channel2': '0',
            'mipi_lpwm_enable': LaunchConfiguration('mipi_lpwm_enable'),
            'mipi_camera_calibration_file_path':  LaunchConfiguration('mipi_camera_calibration_file_path'),
            'mipi_gdc_bin_file': './sc230ai_gdc.bin',
            'mipi_rotation': LaunchConfiguration('mipi_rotation'),
            'mipi_cal_rotation': LaunchConfiguration('mipi_cal_rotation'),
            'mipi_gdc_enable': LaunchConfiguration('mipi_gdc_enable'),
            'mipi_frame_ts_type': 'sensor'
        }.items()
    )

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_combine_raw',
            'codec_pub_topic': '/image_combine_jpeg'
        }.items()
    )
    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_combine_jpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )
    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    return LaunchDescription([
        # 启动零拷贝环境配置node
        mipi_lpwm_enable_arg,
        mipi_camera_calibration_file_path_arg,
        mipi_image_width_arg,
        mipi_image_height_arg,
        mipi_rotation_arg,
        mipi_cal_rotation_arg,
        mipi_gdc_enable_arg,
        mipi_image_framerate_arg,
        shared_mem_node,
        # image publish
        mipi_node,
        # image codec
        jpeg_codec_node,
        # web display
        web_node
    ])
