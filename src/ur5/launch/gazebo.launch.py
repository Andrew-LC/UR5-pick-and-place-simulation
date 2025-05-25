from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # Package directories
    ur5_controller_dir = FindPackageShare('ur5')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    rviz_config = os.path.join(get_package_share_directory('ur5') + '/config/ur_rviz.rviz')

    # Paths
    world_path = os.path.join(get_package_share_directory('ur5') + '/world' + '/my_world.sdf')
    bridge_config_path = PathJoinSubstitution([get_package_share_directory('ur5'), 'config', 'bridge_config.yaml'])

    # Process URDF from Xacro
    urdf = xacro.process_file(get_package_share_directory('ur5') + '/urdf/ur5.urdf.xacro').toxml()
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )

    gz_args_string = ['-r', '-v', '4', str(world_path), '--physics-engine', 'gz-physics-bullet-featherstone-plugin']
    gz_args_combined = ' '.join(gz_args_string)

    print(gz_args_combined)

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
            '-z', '0.75',
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

    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/cam_1/image'],
    )

    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/cam_1/depth_image'],
    )

    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        robot_description,
        joint_state_pub,
        gazebo,
        bridge,
        camera_bridge_image,
        camera_bridge_depth,
        spawn_entity,
        rviz
    ])
