from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='decision_maker', executable='mock_nav_server',
             name='mock_nav_server', output='screen'),
        Node(package='decision_maker', executable='mock_grasp_server',
             name='mock_grasp_server', output='screen'),
    ])
