from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the existing control launch file.
    diffdrive_share = get_package_share_directory("diffdrive_bot")

    control_launch = os.path.join(
        diffdrive_share,
        "launch",
        "control.launch.py"
    )

    # Allow the user to choose the square's side length.
    side_length_arg = DeclareLaunchArgument(
        "side_length",
        default_value="0.5",
        description="Length of each side of the square in meters"
    )

    # Start the existing Gazebo and robot control system.
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )

    # Start the rotation action server.
    rotate_server = Node(
        package="my_robot_nodes",
        executable="rotate_server",
        name="rotate_server",
        output="screen"
    )

    # Start the square-driving action client.
    square_driver = Node(
        package="my_robot_nodes",
        executable="square_driver",
        name="square_driver",
        output="screen",
        parameters=[{
            "side_length": LaunchConfiguration("side_length")
        }]
    )

    return LaunchDescription([
        side_length_arg,
        control,
        rotate_server,
        square_driver
    ])
