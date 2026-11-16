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

# 异常模拟测试入口：在通用 mipi_cam.launch.py 之上暴露 fault_* 参数。
#
# 用法示例：
#   # 同时注入 IMU 停顿 + 图像时间戳过旧（默认周期 10s / 停顿 1s / drift -250ms）
#   ros2 launch mipi_cam mipi_cam_fault_inject.launch.py \
#       fault_inject:="IMU_STALLED,IMAGE_STALE_OR_FUTURE_TIMESTAMP"
#
#   # 图像永久停顿（验证下游的持续报警与恢复）
#   ros2 launch mipi_cam mipi_cam_fault_inject.launch.py \
#       fault_inject:="IMAGE_STALLED" fault_image_stall_duration_ms:=0
#
#   # IMU 时间戳超前（+150ms）
#   ros2 launch mipi_cam mipi_cam_fault_inject.launch.py \
#       fault_inject:="IMU_STALE_OR_FUTURE_TIMESTAMP" fault_ts_drift_mode:=future
#
# 注入期间：
#   - 状态话题 /mipi_cam_fault_status（std_msgs/String）：激活字段集合变化时
#     发布，内容为逗号分隔列表（空串=全部恢复），可作测试的 ground truth
#   - 注入动作有节流 WARN 日志（[fault-inject] ...）
# 合法字段：IMU_STALLED / IMU_STALE_OR_FUTURE_TIMESTAMP /
#           IMU_TIMESTAMP_NOT_INCREASING / IMAGE_STALLED /
#           IMAGE_STALE_OR_FUTURE_TIMESTAMP（非法值节点拒绝启动）

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fault_inject',
            default_value='',
            description='comma separated fault list: IMU_STALLED, '
                        'IMU_STALE_OR_FUTURE_TIMESTAMP, '
                        'IMU_TIMESTAMP_NOT_INCREASING, IMAGE_STALLED, '
                        'IMAGE_STALE_OR_FUTURE_TIMESTAMP'),
        DeclareLaunchArgument(
            'fault_ts_drift_mode',
            default_value='stale',
            description='stale(-250ms) or future(+150ms)'),
        DeclareLaunchArgument(
            'fault_imu_stall_period_s',
            default_value='10.0',
            description='imu stall period seconds, 0 = inject once'),
        DeclareLaunchArgument(
            'fault_imu_stall_duration_ms',
            default_value='1000',
            description='imu stall duration ms, 0 = permanent silence'),
        DeclareLaunchArgument(
            'fault_image_stall_period_s',
            default_value='10.0',
            description='image stall period seconds, 0 = inject once'),
        DeclareLaunchArgument(
            'fault_image_stall_duration_ms',
            default_value='1000',
            description='image stall duration ms, 0 = permanent silence'),
        # 通用参数透传（分辨率/帧率等按 mipi_cam.launch.py 默认，可覆盖）
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
            'mipi_io_method',
            default_value='shared_mem',
            description='ros or shared_mem'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': LaunchConfiguration('mipi_image_width'),
                'mipi_image_height': LaunchConfiguration('mipi_image_height'),
                'mipi_image_framerate': LaunchConfiguration('mipi_image_framerate'),
                'mipi_io_method': LaunchConfiguration('mipi_io_method'),
                'fault_inject': LaunchConfiguration('fault_inject'),
                'fault_ts_drift_mode': LaunchConfiguration('fault_ts_drift_mode'),
                'fault_imu_stall_period_s': LaunchConfiguration('fault_imu_stall_period_s'),
                'fault_imu_stall_duration_ms': LaunchConfiguration('fault_imu_stall_duration_ms'),
                'fault_image_stall_period_s': LaunchConfiguration('fault_image_stall_period_s'),
                'fault_image_stall_duration_ms': LaunchConfiguration('fault_image_stall_duration_ms'),
            }.items()
        )
    ])
