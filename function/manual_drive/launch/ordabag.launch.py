# main/launch/xycar_all.launch.py

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # xycar_cam 패키지의 launch 파일 포함
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_cam'),
                'launch',
                'xycar_cam.launch.py'
            )
        )
    )


    # xycar_ultrasonic 패키지의 launch 파일 포함
    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_ultrasonic'),
                'launch',
                'xycar_ultrasonic.launch.py'
            )
        )
    )

    # image_resize 패키지의 resize_node 실행
    resize_node = Node(
        package='image_resize',
        executable='resize_node',
        name='resize_node',
        output='screen'
    )

    # xycar_lidar 패키지의 launch 파일 포함
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_lidar'),
                'launch',
                'xycar_lidar.launch.py'
            )
        )
    )

    return LaunchDescription([
        cam_launch,
        ultrasonic_launch,
        resize_node,
        lidar_launch,
    ])
