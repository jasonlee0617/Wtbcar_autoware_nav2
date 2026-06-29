import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("rslidar_sdk"), "config", "config.yaml"
    )

    return LaunchDescription([
        # 1. 静态 TF 变换 (base_link -> laser)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_lidar',
        #     arguments=['0.4', '0', '1.5', '0', '0', '0', 'base_link', 'laser'],
        # ),

        # # 2. 点云转换节点 (修复了参数语法错误)
        # Node(
        #     package="wtb_common_sensor_launch",
        #     executable="point_cloud_transformer",
        #     output="screen",
        #     parameters=[
        #         {"use_height_filter": False},  # 建议使用 Python 原生布尔值
        #         {"use_sim_time": False}        # ✅ 修复：将逗号改为冒号，使用布尔值
        #     ],
        # ),

        # 3. 速腾雷达驱动节点
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": config_path}],
        )
    ])