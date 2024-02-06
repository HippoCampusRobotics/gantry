from ament_index_python import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    pkg_path = get_package_share_path('gantry')
    path = str(pkg_path / 'config/path_follower.yaml')
    action = DeclareLaunchArgument('path_follower_config_file',
                                   default_value=path)
    launch_description.add_action(action)


def create_path_follower_node():
    return Node(
        executable='path_follower',
        package='gantry',
        namespace='gantry',
        parameters=[
            LaunchConfiguration('path_follower_config_file'),
        ],
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        create_path_follower_node(),
    ]
    for action in actions:
        launch_description.add_action(action)

    return launch_description
