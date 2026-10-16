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

# S600 双目双码流 launch：左右两路各出主码流+子码流（共四路）。
# sensor 走 MIPI 直连双目(sc230ai, channel 2/0)，dual_combine=0 只出左右图不做拼接。
# stream_mode: 0=子码流同样GDC矫正；1=主码流矫正、子码流仅旋转（参数可覆盖）。
#
# ⚠ web 展示设计（非缺陷）：两个 websocket 节点只订阅 image_left/sub_image_left —— 右路
# 采集与发布完全正常（ros2 topic hz /image_right_raw 验证），web 想看右目时把下方
# websocket_image_topic 改成 image_right_jpeg 即可。
# 日志观察点（log_level:=info 才可见，默认 warn 会滤掉）：
#   create post pym node ok ×2（hw 值必须不同，双目预期 0 和 2）
#   gdc bridge fd init ×2 / PYM+GDC+PYM streams active ×2
# 实测（2026-10-16，vcon@4/5 两颗 sc132gs）：hw:0/hw:2、slot:5/7、四路 30fps 全部达标。

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mipi_cam'),
        "lib/mipi_cam/config/")

    mipi_lpwm_enable_arg = DeclareLaunchArgument(
        'mipi_lpwm_enable',
        default_value='False',
        description='mipi lpwm enable')

    mipi_camera_calibration_file_path_arg = DeclareLaunchArgument(
        'mipi_camera_calibration_file_path',
        default_value=os.path.join(config_path, 'SC230ai_dual_calibration.yaml'),
        description='mipi camera calibration file path')

    mipi_image_width_arg = DeclareLaunchArgument(
        'mipi_image_width',
        default_value='1088',
        description='mipi width (sc132gs native)')

    mipi_image_height_arg = DeclareLaunchArgument(
        'mipi_image_height',
        default_value='1280',
        description='mipi height (sc132gs native)')

    mipi_sub_image_width_arg = DeclareLaunchArgument(
        'mipi_sub_image_width',
        default_value='544',
        description='mipi sub stream image width')

    mipi_sub_image_height_arg = DeclareLaunchArgument(
        'mipi_sub_image_height',
        default_value='640',
        description='mipi sub stream image height')

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
        default_value='30.0',
        description='mipi camera out image framerate')

    mipi_sub_image_framerate_arg = DeclareLaunchArgument(
        'mipi_sub_image_framerate',
        default_value='30.0',
        description='mipi camera out sub stream framerate')

    mipi_frame_ts_type_arg = DeclareLaunchArgument(
        'mipi_frame_ts_type',
        default_value='sensor',
        description='mipi camera out image frame timestamp type')

    mipi_stream_mode_arg = DeclareLaunchArgument(
        'mipi_stream_mode',
        default_value='0',
        description='0: sub stream rectified by gdc; 1: sub stream rotated only')

    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch/mipi_cam_dual_channel.launch.py')),
        launch_arguments={
            'mipi_image_width': LaunchConfiguration('mipi_image_width'),
            'mipi_image_height': LaunchConfiguration('mipi_image_height'),
            'mipi_sub_image_width': LaunchConfiguration('mipi_sub_image_width'),
            'mipi_sub_image_height': LaunchConfiguration('mipi_sub_image_height'),
            'mipi_image_framerate': LaunchConfiguration('mipi_image_framerate'),
            'mipi_sub_framerate': LaunchConfiguration('mipi_sub_image_framerate'),
            'mipi_io_method': 'ros',
            'device_mode': 'dual',
            'dual_combine': '0',
            'mipi_channel': '2',
            'mipi_channel2': '0',
            'mipi_lpwm_enable': LaunchConfiguration('mipi_lpwm_enable'),
            'mipi_config_path': config_path,
            'mipi_camera_calibration_file_path': LaunchConfiguration('mipi_camera_calibration_file_path'),
            'mipi_rotation': LaunchConfiguration('mipi_rotation'),
            'mipi_cal_rotation': LaunchConfiguration('mipi_cal_rotation'),
            'mipi_gdc_enable': LaunchConfiguration('mipi_gdc_enable'),
            'mipi_stream_mode': LaunchConfiguration('mipi_stream_mode'),
            'mipi_sub_stream_enable': 'True',
            'mipi_frame_ts_type': LaunchConfiguration('mipi_frame_ts_type')
        }.items()
    )

    # nv12->jpeg：主码流左目
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_name': 'jpeg_codec_node',
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'nv12',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_left_raw',
            'codec_pub_topic': '/image_left_jpeg'
        }.items()
    )
    # nv12->jpeg：子码流左目
    sub_jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_name': 'sub_jpeg_codec_node',
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'nv12',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/sub_image_left_raw',
            'codec_pub_topic': '/sub_image_left_jpeg'
        }.items()
    )
    # web：主码流
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_left_jpeg',
            'websocket_channel': '0',
            'websocket_only_show_image': 'True'
        }.items()
    )
    # web：子码流
    sub_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/sub_image_left_jpeg',
            'websocket_channel': '1',
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
        mipi_sub_image_width_arg,
        mipi_sub_image_height_arg,
        mipi_rotation_arg,
        mipi_cal_rotation_arg,
        mipi_gdc_enable_arg,
        mipi_image_framerate_arg,
        mipi_sub_image_framerate_arg,
        mipi_frame_ts_type_arg,
        mipi_stream_mode_arg,
        shared_mem_node,
        # image publish
        mipi_node,
        # image codec
        jpeg_codec_node,
        sub_jpeg_codec_node,
        # web display
        web_node,
        sub_web_node
    ])
