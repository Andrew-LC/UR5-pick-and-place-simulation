from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config").to_moveit_configs()

    params = []
    params.append(moveit_config.robot_description)

    for attr in [
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        getattr(moveit_config, "task_constructor_config", None), 
    ]:
        if attr is None:
            continue
        elif isinstance(attr, (list, tuple)):
            params.extend(attr)
        else:
            params.append(attr)

    pick_place_demo = Node(
        package="ur5_moveit_config",
        executable="moveit",
        output="screen",
        parameters=params,
    )

    return LaunchDescription([pick_place_demo])
