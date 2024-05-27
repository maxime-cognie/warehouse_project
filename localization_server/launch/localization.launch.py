import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    map_file = context.perform_substitution(LaunchConfiguration('map_file'))
    print(f'map file : {map_file}')

    use_sim_time = False if map_file == 'warehouse_map_real.yaml' else True

    map_file_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file_path} 
        ])
    
    amcl_config_file = 'amcl_real_config.yaml' if map_file == 'warehouse_map_real.yaml' else 'amcl_config.yaml'
    amcl_config_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', amcl_config_file)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_yaml])

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
        )

    rviz_config_file = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localization_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
    
    nodes_to_start = [
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz]
    
    return nodes_to_start


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument('map_file', 
        default_value='warehouse_map_sim.yaml', 
        description='map file used by the map_server node')

    return LaunchDescription([
        map_file_arg,
        OpaqueFunction(function=launch_setup)
    ])