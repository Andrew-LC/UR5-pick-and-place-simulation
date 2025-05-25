import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur5_moveit_config").to_moveit_configs()

    # Declare launch arguments
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(DeclareLaunchArgument("allow_trajectory_execution", default_value="true"))
    ld.add_action(DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true"))
    ld.add_action(DeclareLaunchArgument(
        "capabilities",
        default_value=moveit_config.move_group_capabilities["capabilities"]
    ))
    ld.add_action(DeclareLaunchArgument(
        "disable_capabilities",
        default_value=moveit_config.move_group_capabilities["disable_capabilities"]
    ))
    ld.add_action(DeclareLaunchArgument("monitor_dynamics", default_value="false"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # move_group configuration
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": LaunchConfiguration("monitor_dynamics"),
        "use_sim_time": use_sim_time,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # Add move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    ld.add_action(move_group_node)
    return ld
