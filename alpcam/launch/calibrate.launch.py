"""Launch file for stereo calibration using the Alp camera.

Launches the camera driver in reliable mode and the ROS camera_calibration
stereo calibrator with the chessboard parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('alpcam')
    default_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('size', default_value='6x8',
                              description='Chessboard internal corners (cols x rows)'),
        DeclareLaunchArgument('square', default_value='0.0247',
                              description='Chessboard square size in meters'),

        # Camera driver — reliable mode so cameracalibrator can subscribe.
        Node(
            package='alpcam',
            executable='camera_node',
            name='alpcam_camera',
            parameters=[
                default_params,
            ],
            output='screen',
        ),

        # Stereo calibrator from camera_calibration package.
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='stereo_calibrator',
            arguments=[
                '--size', LaunchConfiguration('size'),
                '--square', LaunchConfiguration('square'),
                '--approximate', '0.1',
                '--no-service-check',
            ],
            remappings=[
                ('left', '/alpcam_camera/left/image_raw'),
                ('left_camera', '/alpcam_camera/left'),
                ('right', '/alpcam_camera/right/image_raw'),
                ('right_camera', '/alpcam_camera/right'),
            ],
            output='screen',
        ),
    ])
