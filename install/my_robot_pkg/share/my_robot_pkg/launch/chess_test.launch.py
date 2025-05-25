from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    gui_node = Node(
        package='my_robot_pkg',
        executable='gui',
        name='gui',
    )
    chess_vision_node = Node(
        package='my_robot_pkg',
        executable='chess_vision_node',
        name='chess_vision_node',
    )

    
    return LaunchDescription([
        chess_vision_node,
        gui_node,
    ])