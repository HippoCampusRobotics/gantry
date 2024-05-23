from ament_index_python import get_package_share_path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from hippo_common.launch_helper import LaunchArgsDict


def declare_launch_args(launch_description: LaunchDescription):
    pkg_path = get_package_share_path('gantry')
    path = str(pkg_path / 'config/waypoint_grid.yaml')
    action = DeclareLaunchArgument('waypoint_file', default_value=path)
    launch_description.add_action(action)

    action = DeclareLaunchArgument('measurement_time', default_value='3.0')
    launch_description.add_action(action)


def create_grid_position_node():
    args = LaunchArgsDict()
    args.add('waypoint_file')
    args.add('measurement_time')
    return Node(
        executable='grid_position_control.py',
        package='gantry',
        namespace='gantry',
        parameters=[
            args,
        ],
        output='screen',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        create_grid_position_node(),
    ]
    for action in actions:
        launch_description.add_action(action)

    return launch_description
