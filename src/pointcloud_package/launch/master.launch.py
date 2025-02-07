from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():

    # probably requires realsense install
    launch_args = {'pointcloud.enable': 'true'}
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                'launch',
                "rs_launch.py"
            ])
        ]),
        launch_arguments=launch_args.items()
    )

    node_normals = Node(
        package="pointcloud_package",
        executable="pc_processor_node",
        name="pc_processor_node",
        output="log",
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", config_rviz]
    )

    return LaunchDescription([
        launch_realsense, node_normals, node_rviz
    ])
