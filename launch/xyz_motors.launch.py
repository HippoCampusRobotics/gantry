from launch_ros.actions import Node

from launch import LaunchDescription


def add_xyz_motors_node(launch_description: LaunchDescription):
    action = Node(
        executable='xyz_motors',
        package='gantry',
        namespace='gantry',
        output='screen',
        emulate_tty=True,
    )
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    add_xyz_motors_node(launch_description=launch_description)

    return launch_description
