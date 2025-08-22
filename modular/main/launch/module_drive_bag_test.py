import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) 런치 아규먼트 선언
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='Main node mode parameter'
    )
    mode = LaunchConfiguration('mode')

    # 2) 기존 노드들
    main_node = Node(
        package='main',
        executable='main_node',
        name='main_node',
        output='screen',
        parameters=[{'mode': mode}]
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
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen',
    #     parameters=[{
    #         'dev': '/dev/input/js0',
    #         'deadzone': 0.05,
    #     }]
    # )

    # # 3) 포함할 런치파일 추가

    # # xycar_cam.launch (xml 형식)
    # cam_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('xycar_cam'),
    #             'launch/xycar_cam.launch.py'
    #         )
    #     )
    # )

    # # xycar_lidar.launch.py (python 형식)
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('xycar_lidar'),
    #             'launch/xycar_lidar.launch.py'
    #         )
    #     )
    # )

    # 4) LaunchDescription 반환
    return LaunchDescription([
        mode_arg,
        main_node,
        traffic_node,
        rubbercone_node,
        resize_node,
        lane_node,
        object_node,
        # joy_node,
        # cam_launch,
        # lidar_launch,
    ])
