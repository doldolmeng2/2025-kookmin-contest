# main/launch/main_launch.py

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 시작 모드 지정 명령어
# ros2 launch main main_launch.py initial_mode:=3

def generate_launch_description():
    # 1) 런치 아규먼트 선언: initial_mode (기본값 0)
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='0',
        description='Main node initial_mode parameter'
    )
    initial_mode = LaunchConfiguration('initial_mode')

    # 2) 각 노드 실행 설정
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{'initial_mode': initial_mode}]
    )
    traffic_node = Node(
        package='traffic_light',
        executable='traffic_node',
        name='traffic_node',
        output='screen'
    )
    rubbercone_node = Node(
        package='rubbercone',
        executable='rubbercone_node',
        name='rubbercone_node',
        output='screen'
    )
    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen'
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen'
    )
    object_node = Node(
        package='object_detection',
        executable='object_node',
        name='object_node',
        output='screen'
    )

    return LaunchDescription([
        initial_mode_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_node,
    ])
