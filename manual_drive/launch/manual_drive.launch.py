from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
        
    motor_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_motor'),
                'launch/xycar_motor.launch.py'))
    )

    return LaunchDescription([
        # joy_node: 조이스틱 입력 노드
        motor_include,
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
