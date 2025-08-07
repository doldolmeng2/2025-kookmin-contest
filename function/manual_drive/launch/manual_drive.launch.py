from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
        

    return LaunchDescription([
        # joy_node: 조이스틱 입력 노드
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # joystic.py: 사용자가 만든 노드
        Node(
            package='manual_drive',
            executable='joystic',  # setup.py의 console_scripts에서 등록한 이름
            name='joystic_node',
            output='screen',
        ),
    ])
