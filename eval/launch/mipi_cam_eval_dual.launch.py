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

# 评测专用双目 launch：产品 launch 零改动。
# 参数名与节点 declare_parameter 同名，runner 可直接用用例 YAML 的 params 生成。
# 默认值对应 C04：双目 1088x1280@30 nv12 / ros / sensor ts / combine=1。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('mipi_cam'),
        "lib/mipi_cam/config/")

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config_file_path),
        DeclareLaunchArgument('camera_calibration_file_path', default_value='default'),
        DeclareLaunchArgument('out_format', default_value='nv12'),
        DeclareLaunchArgument('io_method', default_value='ros'),
        DeclareLaunchArgument('image_width', default_value='1088'),
        DeclareLaunchArgument('image_height', default_value='1280'),
        DeclareLaunchArgument('sub_image_width', default_value='544'),
        DeclareLaunchArgument('sub_image_height', default_value='640'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('sub_framerate', default_value='15.0'),
        DeclareLaunchArgument('rotation', default_value='0.0'),
        DeclareLaunchArgument('cal_rotation', default_value='0.0'),
        DeclareLaunchArgument('device_mode', default_value='dual'),
        DeclareLaunchArgument('dual_combine', default_value='1'),
        DeclareLaunchArgument('lpwm_enable', default_value='False'),
        DeclareLaunchArgument('gdc_enable', default_value='True'),
        DeclareLaunchArgument('frame_ts_type', default_value='sensor'),
        DeclareLaunchArgument('link_type', default_value='0'),
        DeclareLaunchArgument('link_port', default_value='0'),
        DeclareLaunchArgument('cal_alpha', default_value='0.0'),
        DeclareLaunchArgument('sub_stream_enable', default_value='False'),
        DeclareLaunchArgument('channel', default_value='0'),
        DeclareLaunchArgument('channel2', default_value='2'),
        DeclareLaunchArgument('imu_type', default_value='icm42688'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('gdc_bin_file', default_value='default'),
        DeclareLaunchArgument('eval_log_enable', default_value='True'),
        DeclareLaunchArgument('eval_log_events', default_value='startup,pub_out,imu_error'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        Node(
            package='mipi_cam',
            executable='mipi_cam',
            output='screen',
            parameters=[
                {"config_path": LaunchConfiguration('config_path')},
                {"camera_calibration_file_path": LaunchConfiguration('camera_calibration_file_path')},
                {"out_format": LaunchConfiguration('out_format')},
                {"io_method": LaunchConfiguration('io_method')},
                {"image_width": LaunchConfiguration('image_width')},
                {"image_height": LaunchConfiguration('image_height')},
                {"sub_image_width": LaunchConfiguration('sub_image_width')},
                {"sub_image_height": LaunchConfiguration('sub_image_height')},
                {"framerate": LaunchConfiguration('framerate')},
                {"sub_framerate": LaunchConfiguration('sub_framerate')},
                {"rotation": LaunchConfiguration('rotation')},
                {"cal_rotation": LaunchConfiguration('cal_rotation')},
                {"device_mode": LaunchConfiguration('device_mode')},
                {"dual_combine": LaunchConfiguration('dual_combine')},
                {"lpwm_enable": LaunchConfiguration('lpwm_enable')},
                {"gdc_enable": LaunchConfiguration('gdc_enable')},
                {"frame_ts_type": LaunchConfiguration('frame_ts_type')},
                {"link_type": LaunchConfiguration('link_type')},
                {"link_port": LaunchConfiguration('link_port')},
                {"cal_alpha": LaunchConfiguration('cal_alpha')},
                {"sub_stream_enable": LaunchConfiguration('sub_stream_enable')},
                {"channel": LaunchConfiguration('channel')},
                {"channel2": LaunchConfiguration('channel2')},
                {"imu_type": LaunchConfiguration('imu_type')},
                {"frame_id": LaunchConfiguration('frame_id')},
                {"gdc_bin_file": LaunchConfiguration('gdc_bin_file')},
                {"eval_log_enable": LaunchConfiguration('eval_log_enable')},
                {"eval_log_events": LaunchConfiguration('eval_log_events')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
