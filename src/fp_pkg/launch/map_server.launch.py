import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    # 카토그래퍼 경로 추가
    carto_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')

    param_dir = '/home/dev/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/humble/waffle_pi.yaml'

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    slam          = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument('map',          default_value='/home/dev/my_map/map2.yaml'),
        # DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam',         default_value='false'),

        # SLAM상태가 아닐때만 실행 
        Node(
            condition=UnlessCondition(slam),
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}]
        ),
        Node(
            condition=UnlessCondition(slam),
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[param_dir, {'use_sim_time': use_sim_time}]
        ),

        # SLAM상태일때만 실행 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(carto_launch_file_dir, 'cartographer.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time, 'use_rviz': 'false'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'navigation_launch.py')),
            condition=UnlessCondition(slam),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'false',
                'use_rviz': 'false',
            }.items(),
        )
    ])