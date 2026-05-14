import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
    print("=== shw3h single camera launch ===")

    config_file_path = os.path.join(
        get_package_prefix('mipi_cam'),
        "lib/mipi_cam/config/")
    print("config_file_path is ", config_file_path)

    # ================================================================
    #  参数声明
    # ================================================================

    # gsml_cfg_file 传空 → read_gsml_config 无内容
    # → gsml_config_.empty() == true → 走 else 分支
    mipi_gsml_cfg_file_arg = DeclareLaunchArgument(
        'mipi_gsml_cfg_file',
        default_value='',
        description='gsml config file (empty = single sensor, else branch)'
    )

    # ★ 关键：video_device → cap_info_.sensor_type
    # 必须与传感器配置文件中 sensor_name 完全一致
    mipi_video_device_arg = DeclareLaunchArgument(
        'mipi_video_device',
        default_value='shw3h_shf3l_std-30fps',
        description='sensor name (must match sensor_name in config)')

    # ★ 关键：link_port → cap_info_.link_port_
    # 决定接在解串器的哪个 link 口 (0-3)
    mipi_link_port_arg = DeclareLaunchArgument(
        'mipi_link_port',
        default_value='2',
        description='GMSL link port (0-3)')

    mipi_image_width_arg = DeclareLaunchArgument(
        'mipi_image_width',
        default_value='1920',
        description='mipi width')

    mipi_image_height_arg = DeclareLaunchArgument(
        'mipi_image_height',
        default_value='1536',
        description='mipi height')

    mipi_image_framerate_arg = DeclareLaunchArgument(
        'mipi_image_framerate',
        default_value='30.0',
        description='mipi camera out image framerate')

    mipi_frame_ts_type_arg = DeclareLaunchArgument(
        'mipi_frame_ts_type',
        default_value='sensor',
        description='timestamp type (sensor/realtime)')

    mipi_lpwm_enable_arg = DeclareLaunchArgument(
        'mipi_lpwm_enable',
        default_value='False',
        description='mipi lpwm enable')

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
        default_value='False',
        description='mipi gdc enable (no calibration for single cam)')

    mipi_camera_calibration_file_path_arg = DeclareLaunchArgument(
        'mipi_camera_calibration_file_path',
        default_value='default',
        description='calibration file path')

    # ================================================================
    #  MIPI Camera Node（通过内层 launch）
    # ================================================================
    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch/mipi_cam_gsml.launch.py')),
        launch_arguments={
            'mipi_gsml_cfg_file': LaunchConfiguration('mipi_gsml_cfg_file'),
            'mipi_video_device': LaunchConfiguration('mipi_video_device'),
            'mipi_image_width': LaunchConfiguration('mipi_image_width'),
            'mipi_image_height': LaunchConfiguration('mipi_image_height'),
            'mipi_image_framerate': LaunchConfiguration('mipi_image_framerate'),
            'mipi_io_method': 'ros',
            'device_mode': 'single',
            'dual_combine': '0',
            'mipi_link_type': '1',            # 1 = GMSL
            'mipi_link_port': LaunchConfiguration('mipi_link_port'),
            'mipi_lpwm_enable': LaunchConfiguration('mipi_lpwm_enable'),
            'mipi_camera_calibration_file_path': LaunchConfiguration('mipi_camera_calibration_file_path'),
            'mipi_gdc_bin_file': '',
            'mipi_rotation': LaunchConfiguration('mipi_rotation'),
            'mipi_cal_rotation': LaunchConfiguration('mipi_cal_rotation'),
            'mipi_gdc_enable': LaunchConfiguration('mipi_gdc_enable'),
            'mipi_frame_ts_type': LaunchConfiguration('mipi_frame_ts_type'),
        }.items()
    )

    # ================================================================
    #  NV12 → JPEG 编码
    # ================================================================
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_raw',
            'codec_pub_topic': '/image_jpeg'
        }.items()
    )

    # ================================================================
    #  WebSocket 显示
    # ================================================================
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )

    # ================================================================
    #  共享内存
    # ================================================================
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_shm'),
                'launch/hobot_shm.launch.py'))
    )

    return LaunchDescription([
        # Arguments
        mipi_gsml_cfg_file_arg,
        mipi_video_device_arg,
        mipi_link_port_arg,
        mipi_image_width_arg,
        mipi_image_height_arg,
        mipi_image_framerate_arg,
        mipi_frame_ts_type_arg,
        mipi_lpwm_enable_arg,
        mipi_rotation_arg,
        mipi_cal_rotation_arg,
        mipi_gdc_enable_arg,
        mipi_camera_calibration_file_path_arg,
        # Nodes
        shared_mem_node,
        mipi_node,
        jpeg_codec_node,
        web_node,
    ])
