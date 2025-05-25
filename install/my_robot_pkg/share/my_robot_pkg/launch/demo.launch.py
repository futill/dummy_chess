# dummy2_can2eth_server.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("orbbec_camera"),
                "launch",
                "astra.launch.py",
            )
        )
    )

    ar_moveit_launch = PythonLaunchDescriptionSource(
        [
            os.path.join(
                get_package_share_directory("dummy_moveit"),
                "launch",
                "demo.launch.py",
            )
        ]
    )
    ar_moveit_args = {
        "include_gripper": "False",
    }.items()
    ar_moveit = IncludeLaunchDescription(
        ar_moveit_launch, launch_arguments=ar_moveit_args
    )

    move_node = Node(
        package='my_robot_pkg',
        executable='arm_move',
        name='arm_move',
    )

    dummy2_can2eth_server_node = Node(
        package='dummy2_can2eth',
        executable='dummy2_can2eth_server',
        name='dummy2_can2eth_server',
    )

    return LaunchDescription([
        dummy2_can2eth_server_node,
        realsense,
        move_node,
        ar_moveit,   # <-- 现在是合法的 IncludeLaunchDescription
    ])
