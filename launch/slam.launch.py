import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name= 'minibot'
    package_dir= get_package_share_directory(package_name) 
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time= DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true, use simulated clock'
    )  

    # Declare the path to files
    joy_params_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'joystick_params.yaml' 
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 
        'config', 
        'slam.rviz' 
    )

    mapper_params_online_async_file = os.path.join(
        package_dir, 
        'config', 
        'mapper_params_online_async.yaml'
    )

    # joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params_file]
    )    

    # teleop node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name= 'teleop_node',
        parameters=[joy_params_file],
        remappings=[('/cmd_vel','joy_vel')]
    )   
    
    # rviz2 node
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='both'
    )
    
    # online_async_slam launch 
    online_async_slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )]), 
                launch_arguments={
                    'slam_params_file': mapper_params_online_async_file,
                    'use_sim_time': use_sim_time
                }.items(),
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('minibot'),
                    'launch',
                    'joystick_teleop.launch.py'
                )])
    )
      
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(declare_use_sim_time)

    # Add the nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(node_rviz2)

    # Add SLAM options
    ld.add_action(online_async_slam)
    ld.add_action(joystick)

    # Generate the launch description and 
    return ld

    