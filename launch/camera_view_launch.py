from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    camera_topic = LaunchConfiguration('camera')
    transport = LaunchConfiguration('transport')
    encoding = LaunchConfiguration('encoding')

    camera_topic_launch_arg = DeclareLaunchArgument(
            'camera',
            default_value='',
            description='The topic that the camera publishes to',
    )
    transport_launch_arg = DeclareLaunchArgument(
            'transport',
            default_value='raw',
            description='The image transport to use (raw, compressed, theora)',
            choices=['raw', 'compressed', 'theora'],
    )
    encoding_launch_arg = DeclareLaunchArgument(
            'encoding',
            default_value='bgr8',
            description='The image encoding to use (bgr8, mono8, bgra8, rgb8, rgba8, mono16)',
            choices=['bgr8', 'mono8', 'bgra8', 'rgb8', 'rgba8', 'mono16'],
    )

    camera_view_node = Node(
            package='camera_view',
            executable='camera_view',
            name='camera_view',
            remappings=[
                ('camera', camera_topic),
            ],
            parameters=[{
                'transport': transport,
                'encoding': encoding,
            }],
    )

    return LaunchDescription([
        camera_topic_launch_arg,
        transport_launch_arg,
        encoding_launch_arg,
        camera_view_node,
    ])
