from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Paths to launch files
    ur5_controller_share = FindPackageShare("ur5_moveit_config").find("ur5_moveit_config")
    gazebo_launch = os.path.join(ur5_controller_share, "launch", "gazebo.launch.py")
    move_group_launch = os.path.join(ur5_controller_share, "launch", "move_group.launch.py")
    moveit_rviz_launch = os.path.join(ur5_controller_share, "launch", "moveit_rviz.launch.py")

    return LaunchDescription([
        # Start Gazebo immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Delay Move Group by 15 seconds
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch)
                )
            ]
        ),

        # Delay RViz by 25 seconds
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_rviz_launch)
                )
            ]
        ),
    ])
