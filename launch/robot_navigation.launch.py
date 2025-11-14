import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    package_name= 'minibot'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')


    declare_use_sim_time= DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true, use simulated clock'
    )  

    declare_map_yaml_file= DeclareLaunchArgument(
        'map_yaml_file',
        default_value='/home/dev/do_an/src/minibot/maps/home.yaml',
    )  

    declare_params_file= DeclareLaunchArgument(
        'params_file',
        default_value='/home/dev/do_an/src/minibot/config/md_nav2_params.yaml',
    ) 

    declare_autostart= DeclareLaunchArgument(
        'autostart',
        default_value='True',
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
        'nav2_rviz2.rviz' 
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='both'
    )

    # nav2_navigation launch 
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                )]), 
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'autostart': autostart,
                }.items()            
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_yaml_file)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)

    # Add the nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(node_rviz2)

    # Add navigation
    ld.add_action(navigation)

    # Generate the launch description and 
    return ld

    