import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

import os

def generate_launch_description():
    pkg_path = launch_ros.substitutions.FindPackageShare(package='rnr_map').find('rnr_map')
    gazebo_path = launch_ros.substitutions.FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    world_path = f'{pkg_path}/world/small_house/small_house.world' 
    default_model_path = f'{pkg_path}/models/robot.urdf'
    default_rviz_config_path = f'{pkg_path}/rviz/config.rviz'
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        parameters = [
            {'use_sim_time' : LaunchConfiguration('use_sim_time')}
        ]
    )
    
    spawn_entity = launch_ros.actions.Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-entity', 'test_bot', '-topic', 'robot_description', '-x', '4.0', '-y', '0.0', '-z', '0.5'],
        output = 'screen'
    )
    
    rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', LaunchConfiguration('rvizconfig')]
    )
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(f'{gazebo_path}/launch/gazebo.launch.py'),
        launch_arguments={
            'verbose':'true',
            'world' : [world_path],
            'on_exit_shutdown': 'True'
        }.items()
    )
    
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[launch.substitutions.EnvironmentVariable('GAZEBO_MODEL_PATH'), f':{pkg_path}/world/small_house/models/']), #[f'{pkg_path}/world/small_house/models/']),
        launch.actions.SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=[launch.substitutions.EnvironmentVariable('GAZEBO_RESOURCE_PATH'), f':{pkg_path}/world/small_house/']),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        launch_gazebo,
        spawn_entity,
        rviz_node
    ]) 