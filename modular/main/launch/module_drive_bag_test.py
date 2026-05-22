# ─────────────────────────────────────────────────────────────────────────────
# module_drive_bag_test.py
#
# 역할: ROS bag 파일 재생 테스트용 런치 파일
#
# module_drive.py와의 차이점:
#   - 하드웨어 드라이버 (카메라, LiDAR) 런치 파일 미포함
#     → bag 파일이 /image_raw, /scan 등 토픽을 직접 재생하기 때문
#   - joy_node, xycar_ultrasonic 미포함 (하드웨어 없음)
#
# 시작되는 노드:
#   main_node, traffic_node, rubbercone_node,
#   resize_node, lane_node, object_node
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── 런치 인수: main_node 초기 모드 ──────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='main_node 초기 주행 모드 (0=TRAFFIC_WAIT)'
    )
    mode = LaunchConfiguration('mode')

    # ── 소프트웨어 노드 (하드웨어 드라이버 제외) ────────────────────────────
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{'mode': mode}],
    )
    traffic_node = Node(
        package='traffic_light',
        executable='traffic_node',
        name='traffic_node',
        output='screen',
    )
    rubbercone_node = Node(
        package='rubbercone',
        executable='rubbercone_node',
        name='rubbercone_node',
        output='screen',
    )
    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen',
    )
    lane_node = Node(
        package='lane_detection',
        executable='lane_node',
        name='lane_node',
        output='screen',
    )
    object_node = Node(
        package='object_detection',
        executable='object_node',
        name='object_node',
        output='screen',
    )

    return LaunchDescription([
        mode_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_node,
    ])
