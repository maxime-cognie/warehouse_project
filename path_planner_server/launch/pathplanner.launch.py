import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    sim = context.perform_substitution(LaunchConfiguration('sim'))

    pkg_share_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')

    if(sim == 'True'):
        nav2_yaml = os.path.join(pkg_share_dir, 'planner_server.yaml')
        controller_yaml = os.path.join(pkg_share_dir, 'controller.yaml')
        bt_navigator_yaml = os.path.join(pkg_share_dir, 'bt.yaml')
        recovery_yaml = os.path.join(pkg_share_dir, 'recovery.yaml')
        cmd_vel_topic = 'diffbot_base_controller/cmd_vel_unstamped'
        use_sim_time = True
    else:
        nav2_yaml = os.path.join(pkg_share_dir, 'planner_server_real.yaml')
        controller_yaml = os.path.join(pkg_share_dir, 'controller_real.yaml')
        bt_navigator_yaml = os.path.join(pkg_share_dir, 'bt_real.yaml')
        recovery_yaml = os.path.join(pkg_share_dir, 'recovery_real.yaml')
        cmd_vel_topic = 'cmd_vel'
        use_sim_time = False

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_yaml]
        )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[controller_yaml],
        remappings=[('/cmd_vel', cmd_vel_topic)]
        )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml]
        )
    
    recovery_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml],
        output='screen'
        )
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}]
        )
    
    rviz_config_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanning.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    nodes_to_start = [
        planner_node,
        controller_node,
        bt_navigator_node,
        recovery_node,
        nav2_lifecycle_manager,
        rviz
    ]

    return nodes_to_start

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('sim', 
        default_value='True', 
        description='Boolean argument to specify if the robot is simulated or not.\nTrue if the robot is simulated and False if working with a real robot.')

    return LaunchDescription([
        sim_arg,
        OpaqueFunction(function=launch_setup)
    ])