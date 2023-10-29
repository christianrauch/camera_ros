from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="640", description="Camera image width"),
            DeclareLaunchArgument("height", default_value="320", description="Camera image height"),
            DeclareLaunchArgument("log_level", default_value="debug", description="Log level (e.g., debug)"),
            # You can add other launch arguments as needed.
            # Run the camera_node with the specified parameters.
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",  # Adjust the output as needed.
                parameters=[
                    {"width": LaunchConfiguration("width")},
                    {"height": LaunchConfiguration("height")},
                    {"mode_width": 2304},
                    {"mode_height": 1296},
                    {"qos_overrides./camera/image_raw.publisher.reliability": "best_effort"},
                    {"qos_overrides./camera/image_raw/compressed.publisher.reliability": "best_effort"},
                    # {"qos_overrides./camera/camera_info.publisher.reliability": "best_effort"},
                ],
                remappings=[],  # Add remappings if necessary.
                arguments=["--log-level", LaunchConfiguration("log_level")],
            ),
            # You can add additional nodes or actions here as needed.
        ]
    )
