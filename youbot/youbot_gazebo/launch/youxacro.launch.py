import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_youbot_model_description = get_package_share_directory('youbot_model_description')
    xacro_file =  os.path.join(pkg_youbot_model_description, 'urdf', 'youbot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description_config.toxml(), "use_sim_time": use_sim_time}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[params]
    )

     # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
      #arguments=['-d', os.path.join(pkg_youbot_rviz, 'config', 'config.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='use sim time if true'),
        DeclareLaunchArgument(
            'rviz', 
            default_value='true',
            description='Open RViz.'),
            
        robot_state_publisher,
        rviz
      
        ])
