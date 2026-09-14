"""col_de 示例的 launch 文件。

注意：运行前需先启动你的 MoveIt（move_group 及机器人控制器），
本 launch 只负责启动 col_de 示例节点。
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("col_de"), "config", "params.yaml")

    demo_node = Node(
        package="col_de",
        executable="demo_waypoints",
        name="col_de_demo",
        output="screen",
        arguments=["--config", config_path],
    )

    return LaunchDescription([demo_node])