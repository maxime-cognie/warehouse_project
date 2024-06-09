from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('localization_server'),
                'launch',
                'localization.launch.py'])
        ]),
        launch_arguments={
            'map_file': 'warehouse_map_sim_keepout.yaml'
        }.items()
    )

pathplanning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_planner_server'),
                'launch',
                'pathplanner.launch.py'])
        ]),
        launch_arguments={
            'filters': 'True'
        }.items()
    )

attach_shelf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('attach_shelf'),
                'launch',
                'attach_shelf_service_server.launch.py'])
        ])
    )

def generate_launch_description():
    return LaunchDescription([
        localization_launch,
        pathplanning_launch,
        attach_shelf_launch])