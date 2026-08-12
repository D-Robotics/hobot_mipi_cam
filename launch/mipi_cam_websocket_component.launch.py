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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    print("using mipi camera")
    # using mipi cam publish image

    mipi_camera_calibration_file_path_arg = DeclareLaunchArgument(
            'mipi_camera_calibration_file_path',
            default_value='default',
            description='mipi camera calibration file path')
    mipi_rotation_arg = DeclareLaunchArgument(
        'mipi_rotation',
        default_value='0.0',
        description='mipi camera out image rotation')
    mipi_cal_rotation_arg = DeclareLaunchArgument(
        'mipi_cal_rotation',
        default_value='0.0',
        description='mipi camera calibration rotation')
    mipi_image_width_arg = DeclareLaunchArgument(
            'mipi_image_width',
            default_value='960',
            description='mipi camera out image width')
    mipi_image_height_arg = DeclareLaunchArgument(
            'mipi_image_height',
            default_value='544',
            description='mipi camera out image height')
    mipi_channel_arg = DeclareLaunchArgument(
            'mipi_channel',
            default_value='0',
            description='mipi camera out image height')

    mipi_lpwm_enable_arg = DeclareLaunchArgument(
            'mipi_lpwm_enable',
            default_value='False',
            description='mipi dual camera lpwm enable')

    mipi_io_method_args = DeclareLaunchArgument(
            'mipi_io_method',
            default_value='ros',
            description='mipi camera out io_method')

    mipi_image_framerate_args = DeclareLaunchArgument(
            'mipi_image_framerate',
            default_value='30.0',
            description='mipi camera out image framerate')

    mipi_image_subframerate_args = DeclareLaunchArgument(
            'mipi_image_subframerate',
            default_value='30.0',
            description='mipi camera out image sub framerate')

    mipi_out_format_args = DeclareLaunchArgument(
            'mipi_out_format',
            default_value='nv12',
            description='mipi camera out format')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', 
        default_value='tros_container',
        description='the name of the container that launches all nodes')

    websocket_channel_args = DeclareLaunchArgument(
        'websocket_channel',
        default_value='0',
        description='websocket channel number')



    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_mjpeg',
            'websocket_channel': LaunchConfiguration('websocket_channel'),
            'websocket_only_show_image': 'True'
        }.items()
    )

    mipi_cmd_group = GroupAction([
        # component container
        Node(
            #condition=False,
            name='tros_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            exec_name='tros_container',
            parameters=[
                # 一定要传入这个参数，否则composition模式下参数传入不到nav2
                {'autostart':True}],
            arguments=['--ros-args', '--log-level', "warn"],
            # prefix=['xterm -e gdb -ex run --args'],
            output='screen')
    ])


    load_composable_nodes = LoadComposableNodes(
        #condition=IfCondition(use_composition),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            
            # mipi
            ComposableNode(
                package='mipi_cam',
                plugin='mipi_cam::MipiCamNode',
                name='MipiCamNode',
                parameters=[
                    {"camera_calibration_file_path": LaunchConfiguration(
                        'mipi_camera_calibration_file_path')},
                    {"out_format": LaunchConfiguration('mipi_out_format')},
                    {"framerate": LaunchConfiguration('mipi_image_framerate')},
                    {"sub_framerate": LaunchConfiguration('mipi_image_subframerate')},
                    {"image_width": LaunchConfiguration('mipi_image_width')},
                    {"image_height": LaunchConfiguration('mipi_image_height')},
                    {"io_method": LaunchConfiguration('mipi_io_method')},
                    {"channel": LaunchConfiguration('mipi_channel')},
                    {"rotation": LaunchConfiguration('mipi_rotation')},
                    {"lpwm_enable": LaunchConfiguration('mipi_lpwm_enable')},
                    {"cal_rotation": LaunchConfiguration('mipi_cal_rotation')},
                ],

                extra_arguments=[{'use_intra_process_comms': True}],
            ),
                
            # 编码节点：畸变矫正后的图编码成 jpeg 格式，用于可视化
            ComposableNode(
                package='hobot_codec',
                plugin='HobotCodecNode',
                name='nv12_code',
                parameters=[
                    {'in_format': 'nv12'},
                    {'in_mode': 'ros'},
                    {'sub_topic': '/image_raw'},  # 畸变矫正后的左图
                    {'out_mode': 'ros'},
                    {'out_format': 'jpeg'},
                    {'pub_topic': '/image_mjpeg'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ]
    )


    return LaunchDescription([
        mipi_camera_calibration_file_path_arg,
        mipi_rotation_arg,
        mipi_cal_rotation_arg,
        mipi_image_width_arg,
        mipi_image_height_arg,
        mipi_channel_arg,
        mipi_lpwm_enable_arg,
        mipi_io_method_args,
        mipi_image_framerate_args,
        mipi_image_subframerate_args,
        mipi_out_format_args,
        declare_container_name_cmd,
        mipi_cmd_group,
        load_composable_nodes,
        websocket_channel_args,
        web_node
    ])
