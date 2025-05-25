from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # Package directories
    ur5_controller_dir = FindPackageShare('ur5_moveit_config')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Paths
    world_path = os.path.join(get_package_share_directory('ur5') + '/world' + '/my_world.sdf')
    bridge_config_path = PathJoinSubstitution([ur5_controller_dir, 'config', 'bridge_config.yaml'])
    urdf_path = PathJoinSubstitution([ur5_controller_dir, 'config', 'ur.urdf.xacro'])
    controller_config_path = PathJoinSubstitution([ur5_controller_dir, 'config', 'ros2_controllers.yaml'])

    # Process URDF from Xacro
    urdf = xacro.process_file(get_package_share_directory('ur5_moveit_config') + '/config/ur.urdf.xacro').toxml()
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )

    gz_args_string = ['-r', '-v', '4', str(world_path), '--physics-engine', 'gz-physics-bullet-featherstone-plugin']
    gz_args_combined = ' '.join(gz_args_string)


    # Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': gz_args_combined
        }.items()
    )

    # Spawn UR5 in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur5',
            '-x', '-0.4',
            '-z', '0.75'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--param-file',
            controller_config_path],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'Manipulator_controller',
            '--param-file',
            controller_config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        robot_description,
        gazebo,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
        bridge,
        spawn_entity,
    ])
