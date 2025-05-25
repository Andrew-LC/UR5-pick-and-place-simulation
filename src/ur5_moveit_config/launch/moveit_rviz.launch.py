from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur5_moveit_config").to_moveit_configs()
    # Declare 'use_sim_time' argument
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Declare 'rviz_config' argument
    declare_rviz_config = DeclareLaunchArgument(
        name="rviz_config",
        default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        description="RViz configuration file"
    )

    # Use LaunchConfiguration to access 'use_sim_time' and 'rviz_config'
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Create RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Assemble launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config,
        rviz_node,
    ])
