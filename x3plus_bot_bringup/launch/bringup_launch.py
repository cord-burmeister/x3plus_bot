# Copyright (c) 2025 Cord Burmeister
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use the core robot system."""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



#region Convert XACRO file

def evaluate_xacro(context, *args, **kwargs):
    """
    Evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be
     included in launch description    
    Method is converting the XACRO  description into a URDF file format. 
    XARCO Allows some parameters and programming in the robot description
    This is implemented as OpaqueFunction to process description ad publish it. 
    """

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('x3plus_description'), 'urdf', 'yahboomcar_X3plus.urdf.xacro')

    #robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = xacro.process_file(xacro_file, 
            mappings={  
                }).toxml()

    robot_state_publisher_node = Node(
       package='robot_state_publisher',
       executable='robot_state_publisher',
       name='robot_state_publisher',
       output='both',
       parameters=[{
        'robot_description': robot_description_config
      }])

    return [robot_state_publisher_node]

#endregion

def generate_launch_description():
#region  Get the launch directories
    pkg_bringup = get_package_share_directory('x3plus_bringup')
    bringup_launch_dir = os.path.join(pkg_bringup, 'launch')
    pkg_nav2 = get_package_share_directory('x3plus_nav2')
    pkg_wrapper = get_package_share_directory('x3plus_wrapper')
    pkg_sllidar_ros2 = get_package_share_directory('sllidar_ros2')
    
#endregion 

#region  Create the launch configuration variables
    # args that can be set from the command line or a default will be used
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
#endregion

#region  Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')    
#endregion 

#region  Declare the launch arguments

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_nav2, 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_nav2, 'config', 'nav2_params-DWBLocalPlanner.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav2, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_home, 'config', 'nav2_params-SmacStateLattice.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

# decided to try to isolate which part of the launch chain was going belly-up, by launching just the Nav2 stuff with ros2 launch nav2_bringup navigation_launch.py. This appears to start-up nicely but eventually gets
# [ERROR] [controller_server-1]: process has died [pid 2529, exit code -4, cmd '/opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args --log-level info --ros-args -p use_sim_time:=False --params-file /tmp/launch_params_j3qiyfo_ -r /tf:=tf -r /tf_static:=tf_static -r cmd_vel:=cmd_vel_nav'].
# https://robotics.stackexchange.com/questions/114131/trouble-getting-nav2-getting-started-example-to-run
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_nav2, 'config', 'nav2_params-MPPIController.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_nav2, 'config', 'nav2_default_view.rviz') ,
        description='Full path to the RVIZ config file to use')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav2, 'maps', 'map.yaml'),
        description='Full path to the ROS2 map file to use for navigation')

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to start the x3plus logic')

#endregion


    # Instead of using IncludeLaunchDescription, directly launch the sllidar_node with remappings:
    lidar_cmd = Node (
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800, 
            'frame_id': 'laser_link',
            'inverted': False, 
            'angle_compensate': True, 
            'scan_mode': 'Standard',
        }], 
        remappings=[('scan', 'scan_raw')]
    )

    # Declare the launch arguments for the wrapper node
    wrapper_cmd= Node(
        package='x3plus_wrapper',
        executable='Mecanum_driver_X3Plus',
    )
    # # Declare the launch arguments for the wrapper node
    # wrapper_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_wrapper, 'launch', 'drive_bringup_X3Plus_launch.py')),
    #     launch_arguments={'frame_id': 'laser_link'}.items())



    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_nav2': use_nav2,
                          'use_respawn': use_respawn}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_nav2_cmd)

    # ld.add_action(declare_robot_name_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any simulation  actions
    ld.add_action(OpaqueFunction(function=evaluate_xacro))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(lidar_cmd)
    ld.add_action(wrapper_cmd)
    ld.add_action(bringup_cmd)
    return ld
