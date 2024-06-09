from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    approach_service_server_node = Node(
            package="attach_shelf",
            executable="approach_service_server_node",
            name="approach_service_server_node",
            output="screen",
            parameters=[{'use_sim_time': True}])
    
    return LaunchDescription([
        approach_service_server_node  
    ])