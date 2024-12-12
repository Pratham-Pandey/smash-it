import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('new_ball'))
    xacro_file = os.path.join(pkg_path,'description','ball.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # ROBOT STATE PUBLISHER    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # RVIZ
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),                   
                    #launch_arguments={'world': "./src/ball_spawner/worlds/base_world.world"}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ball_bot'],
                        output='screen')
    

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        rviz,
        gazebo,
        spawn_entity,
    ])
