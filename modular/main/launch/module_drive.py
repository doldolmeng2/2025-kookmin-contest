# ─────────────────────────────────────────────────────────────────────────────
# module_drive.py
#
# 역할: 자율주행 전체 시스템 런치 파일 (실제 Xycar 하드웨어 구동용)
#
# 시작되는 노드 목록:
#   main_node       - 상태 머신 및 모터 제어 (main 패키지)
#   traffic_node    - 신호등 검출 (traffic_light 패키지)
#   rubbercone_node - 라바콘 구간 LiDAR 오프셋 (rubbercone 패키지)
#   resize_node     - 카메라 영상 640×360 리사이즈 (image_resize 패키지)
#   lane_node       - 차선 검출 및 오프셋 발행 (lane_detection 패키지)
#   object_node     - 장애물 검출 (object_detection 패키지)
#   joy_node        - Xbox 컨트롤러 입력 (joy 패키지)
#
# 포함되는 런치 파일:
#   xycar_cam.launch.py       - 카메라 드라이버
#   xycar_lidar.launch.py     - LiDAR 드라이버
#   xycar_ultrasonic.launch.py- 초음파 드라이버
#
# 런치 인수:
#   mode (기본값: 0) - main_node 초기 주행 모드
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
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

    # ── 소프트웨어 노드 ──────────────────────────────────────────────────────
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
    # Xbox 컨트롤러: /dev/input/js0 장치, deadzone 0.05
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
        }],
    )

    # ── 하드웨어 드라이버 런치 파일 ──────────────────────────────────────────
    # 카메라: AnyLaunchDescriptionSource (xml/py 모두 지원)
    cam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_cam'),
                'launch/xycar_cam.launch.py'
            )
        )
    )
    # LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_lidar'),
                'launch/xycar_lidar.launch.py'
            )
        )
    )
    # 초음파
    ultrasonic_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_ultrasonic'),
                'launch/xycar_ultrasonic.launch.py'
            )
        )
    )

    return LaunchDescription([
        mode_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_node,
        joy_node,
        cam_launch,
        lidar_launch,
        ultrasonic_launch,
    ])
