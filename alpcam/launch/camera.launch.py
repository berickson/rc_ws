"""Launch file for the Alp stereo camera driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('alpcam')
    default_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    left_cal = os.path.join(pkg_share, 'config', 'left.yaml')
    right_cal = os.path.join(pkg_share, 'config', 'right.yaml')

    # Only set calibration paths if the files exist.
    cal_overrides = {}
    if os.path.isfile(left_cal):
        cal_overrides['left_camera_info_url'] = left_cal
    if os.path.isfile(right_cal):
        cal_overrides['right_camera_info_url'] = right_cal

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
                cal_overrides,
            ],
            output='screen',
        ),

        # Stereo rectification — produces image_rect and disparity topics.
        ComposableNodeContainer(
            name='stereo_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    remappings=[
                        ('left/image_rect', '/alpcam_camera/left/image_rect'),
                        ('left/camera_info', '/alpcam_camera/left/camera_info'),
                        ('right/image_rect', '/alpcam_camera/right/image_rect'),
                        ('right/camera_info', '/alpcam_camera/right/camera_info'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='left_rectify',
                    namespace='alpcam_camera/left',
                    remappings=[
                        ('image', 'image_raw'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='right_rectify',
                    namespace='alpcam_camera/right',
                    remappings=[
                        ('image', 'image_raw'),
                    ],
                ),
            ],
            output='screen',
        ),
    ])
