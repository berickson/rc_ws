"""Launch file for the Alp stereo camera driver."""

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
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the camera parameters YAML file',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='',
            description='Override resolution (e.g. 3200x1200, 2560x960, 1280x480)',
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='0',
            description='Override frame rate (0 = use value from params file)',
        ),
        Node(
            package='alpcam',
            executable='camera_node',
            name='alpcam_camera',
            parameters=[
                LaunchConfiguration('params_file'),
            ],
            output='screen',
        ),
    ])
