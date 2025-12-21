import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paket Yollarını Tanımla
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. TurtleBot3 Modelini Ayarla 
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    # 3. Gazebo Simülasyonu (TurtleBot3 Dünyası) [cite: 12, 21]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 4. SLAM Toolbox (Haritasız Navigasyon İçin Şart) [cite: 23]
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 5. Navigasyon (Nav2 Stack) [cite: 22]
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        set_tb3_model,
        gazebo,
        slam,
        navigation
    ])