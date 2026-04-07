from ament_index_python.packages import get_package_share_directory
import os
import sys
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Add utils path (same as example.launch.py does)
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))
from launch_utils import load_yaml  # noqa: E402


def generate_nodes(context):
    # Include the standard example launcher which starts each robot namespace
    nodes = []
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'launch', 'example.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': LaunchConfiguration('robot_config_file').perform(context),
                'controller_name': LaunchConfiguration('controller_name').perform(context),
            }.items(),
        )
    )

    # Load config and take the first two robot entries (order from the YAML file)
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    configs = load_yaml(config_file)
    items = list(configs.items())
    if len(items) < 2:
        return nodes

    _, config_a = items[0]
    _, config_b = items[1]

    # -------------------------------------------------------------------------
    # TF tree structure (symmetric design)
    #
    #                          base   (global world frame)
    #                         /    \
    #                   NS1_base   NS2_base
    #                      |           |
    #               NS1_fr3_link0  NS2_fr3_link0
    #                    ...            ...
    #               NS1_fr3_hand_tcp  NS2_fr3_hand_tcp
    #
    # Both robots are connected to 'base' via their own intermediate base frame
    # (NS1_base / NS2_base), configured via the 'connected_to' field in the
    # robot YAML config.  This makes both arms symmetric in the TF tree and
    # allows each robot's mounting offset to be changed independently.
    #
    # Robot A (NS1): identity transform  — sits at the world origin
    # Robot B (NS2): offset transform    — tx=1.6 m, yaw=180° (faces robot A)
    # -------------------------------------------------------------------------

    # -- Robot A (NS1): identity transform base → NS1_base -------------------
    # NS1 sits at the world origin, so the transform is a pure identity.
    # 'connected_to' in the YAML should be set to 'NS1_base'.
    connected_a = config_a.get('connected_to', None)
    if not connected_a:
        # Fallback: derive the URDF root link name from arm_prefix / arm_id
        raw_arm_prefix = str(config_a.get('arm_prefix', '') or '')
        arm_id = str(config_a.get('arm_id') or 'fr3') or 'fr3'
        modified_prefix = f"{raw_arm_prefix}_" if raw_arm_prefix else ''
        connected_a = f"{modified_prefix}{arm_id}_link0"

    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_robot_a',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',          # identity translation
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',  # identity rotation
                '--frame-id', 'base',
                '--child-frame-id', connected_a,
            ],
            output='screen',
        )
    )

    # -- Robot B (NS2): offset transform base → NS2_base ---------------------
    # NS2 is placed 1.6 m along X and rotated 180° around Z so both robots
    # face each other.  'connected_to' in the YAML should be set to 'NS2_base'.
    tx = 1.26
    ty = 0.0
    tz = 0.01
    yaw = math.pi  # 180° — robots face each other along the X axis

    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)   # sin(90°) = 1.0
    qw = math.cos(yaw / 2.0)   # cos(90°) = 0.0

    connected_b = config_b.get('connected_to', None)
    if not connected_b:
        raw_arm_prefix = str(config_b.get('arm_prefix', '') or '')
        arm_id = str(config_b.get('arm_id') or 'fr3') or 'fr3'
        modified_prefix = f"{raw_arm_prefix}_" if raw_arm_prefix else ''
        connected_b = f"{modified_prefix}{arm_id}_link0"

    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_robot_b',
            arguments=[
                '--x', str(tx), '--y', str(ty), '--z', str(tz),
                '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                '--frame-id', 'base',
                '--child-frame-id', connected_b,
            ],
            output='screen',
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'custom_franka_both.config.yaml'
            ]),
            description='Path to the robot configuration YAML file',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Name of the controller to spawn (required, no default)',
        ),
        OpaqueFunction(function=generate_nodes),
    ])