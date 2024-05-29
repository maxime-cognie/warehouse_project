import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    cartographer_config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument('config_file', default_value='cartographer_sim.lua')

    use_sim_time = True

    cartographer_node = Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', cartographer_config_file])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    rviz_config_file = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'cartographer_rviz_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
    
    return LaunchDescription([
        config_file_arg,
        cartographer_node,
        occupancy_grid_node,
        rviz
    ])