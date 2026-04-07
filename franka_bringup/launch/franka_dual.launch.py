from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import math


def franka_robot(namespace, robot_ip, connected_to):
    return GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("franka_bringup"),
                    "launch",
                    "franka.launch.py",
                ])
            ),
            launch_arguments={
                "robot_ip": robot_ip,
                "connected_to": connected_to
            }.items(),
        ),

        # Load controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "my_cartesian_velocity_controller",
                "--inactive",
            ],
            output="screen",
        ),
    ])


def generate_launch_description():
    # Create the two robot groups
    group_a = franka_robot("ns1", "172.16.2.2", "base")
    group_b = franka_robot("ns2", "172.16.3.2", "NS2_base")

    # Hardcoded transform offset (1.6 m on X and 180 deg yaw so robots face each other)
    tx = 1.6
    ty = 0.0
    tz = 0.02
    yaw = math.pi
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    # The frame on the second robot that is used as the attachment point
    connected_b = "NS2_base"

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_robot_b',
        arguments=[
            str(tx), str(ty), str(tz),
            str(qx), str(qy), str(qz), str(qw),
            'base', connected_b,
        ],
        output='screen',
    )

    return LaunchDescription([
        group_a,
        group_b,
        static_tf,
    ])