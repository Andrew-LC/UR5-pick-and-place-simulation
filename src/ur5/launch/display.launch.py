from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    urdf = xacro.process_file(get_package_share_directory('ur5') + '/urdf/ur5.urdf.xacro').toxml()
    rviz_config = os.path.join(get_package_share_directory('ur5') + '/config/ur_rviz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]  
        )
    ])
