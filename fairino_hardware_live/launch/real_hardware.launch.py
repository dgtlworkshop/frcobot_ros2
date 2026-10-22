from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory("fairino_hardware_live"))
    description = xacro.process_file(
        str(share / "urdf" / "fairino_real.urdf.xacro"),
        mappings={
            "model": LaunchConfiguration("model").perform(context),
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
        },
    ).toxml()
    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": description},
                str(share / "config" / "controllers.yaml"),
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["fairino_controller", "--inactive"],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip", description="Real robot controller IPv4 address"
            ),
            DeclareLaunchArgument(
                "model",
                default_value="fairino20_v6",
                choices=[
                    "fairino3_v6",
                    "fairino3_mt_v6",
                    "fairino5_v6",
                    "fairino10_v6",
                    "fairino16_v6",
                    "fairino20_v6",
                    "fairino30_v6",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
