from launch.launch_description import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description with for the camera node and a visualiser.

    Returns
    -------
        LaunchDescription: the launch description

    """
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                parameters=[{
                    "camera": 0,
                    "width": 640,
                    "height": 480,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_view',
                plugin='image_view::ImageViewNode',
                remappings=[('/image', '/camera/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return LaunchDescription([container])
