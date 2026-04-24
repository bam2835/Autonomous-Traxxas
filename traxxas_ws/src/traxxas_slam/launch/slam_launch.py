import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    cartographer_config_dir = os.path.join(
        get_package_share_directory('traxxas_slam'), 'config')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rplidar_ros'),
                             'launch', 'rplidar_a1_launch.py')
            ])
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='traxxas_slam',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            remappings=[
                ('imu', '/imu/data'),
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer.lua'
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{
		'resolution': 0.05,
		'publish_period_sec': 0.25
		}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
