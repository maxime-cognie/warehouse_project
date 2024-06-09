from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    obs_arg = DeclareLaunchArgument('obstacle', default_value='0.3')
    deg_arg = DeclareLaunchArgument('degrees', default_value='-90')
    fa_arg = DeclareLaunchArgument('final_approach', default_value='True')

    pre_approach_node_v2 = Node(  
        package="attach_shelf",
        executable="pre_approach_node_v2",
        name="pre_approach_node_v2",
        parameters=[
            {'obstacle': obstacle,'degrees': degrees, 'final_approach': final_approach}],
        output="screen")
    
    
    approach_service_server_node = Node(
            package="attach_shelf",
            executable="approach_service_server_node",
            name="approach_service_server_node",
            output="screen")
    
    return LaunchDescription([
        obs_arg,
        deg_arg,
        fa_arg,
        pre_approach_node_v2,
        approach_service_server_node  
    ])