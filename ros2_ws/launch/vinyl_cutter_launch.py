from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vinyl_cutter',
            namespace='vinyl_cutter',
            executable='vinyl_cutter',
            name='vinyl_cutter'
        ),
        Node(
            package='vinyl_cutter',
            namespace='limit_switch',
            executable='limit_switch',
            name='limit_switch'
        ),
        Node(
            package='vinyl_cutter',
            namespace='r_motor',
            executable='r_motor',
            name='r_motor'
        ),
        Node(
            package='vinyl_cutter',
            namespace='speakers',
            executable='speakers',
            name='speakers'
        ),
        Node(
            package='vinyl_cutter',
            namespace='theta_motor',
            executable='theta_motor',
            name='theta_motor'
        ),
        Node(
            package='vinyl_cutter',
            namespace='web_interface',
            executable='web_interface',
            name='web_interface'
        ),

    ])