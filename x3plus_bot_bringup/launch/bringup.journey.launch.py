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
from launch.actions import (
	DeclareLaunchArgument,
	ExecuteProcess,
	IncludeLaunchDescription,
	OpaqueFunction,
	SetLaunchConfiguration
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
	LaunchConfiguration,
	PythonExpression,
	PathJoinSubstitution,
	TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# region Convert XACRO file


def evaluate_xacro(context, *args, **kwargs):
	"""
	Evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be
	 included in launch description
	Method is converting the XACRO  description into a URDF file format.
	XARCO Allows some parameters and programming in the robot description
	This is implemented as OpaqueFunction to process description ad publish it.
	"""

	# Use xacro to process the file
	xacro_file = os.path.join(
		get_package_share_directory("x3plus_description"),
		"urdf",
		"yahboomcar_X3plus.urdf.xacro",
	)

	# robot_description_config = xacro.process_file(xacro_file)
	robot_description_config = xacro.process_file(xacro_file, mappings={}).toxml()
	use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
		"true",
		"1",
	)

	robot_state_publisher_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		name="robot_state_publisher",
		output="both",
		parameters=[
			{
				"robot_description": robot_description_config,
				"use_sim_time": use_sim_time,
			}
		],
	)

	return [robot_state_publisher_node]


# endregion

def derive_configs(context, *args, **kwargs):
	use_case = LaunchConfiguration("use_case").perform(context)
	slam = LaunchConfiguration("slam").perform(context)
	use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
	mode = LaunchConfiguration("mode").perform(context)

	if (mode == "companion"):
		use_sim_time = "false"
	elif (mode == "simulation"):
		use_sim_time = "true"
	elif (mode == "hil"):
		use_sim_time = "false"
	elif (mode == "robot"):
		use_sim_time = "false"
	else:
		raise ValueError(f"Unsupported mode '{mode}'")

	if (use_case == "slam"):
		slam = "True"
	elif (use_case == "explore-frontier"):
		slam = "True"
	elif (use_case == "explore"):
		slam = "True"
	else:
		raise ValueError(f"Unsupported use_case '{use_case}'")
	return [SetLaunchConfiguration("slam", slam), SetLaunchConfiguration("use_sim_time", use_sim_time)]


def generate_launch_description():
	# region  Get the launch directories
	bringup_dir = get_package_share_directory("nav2_bringup")
	launch_dir = os.path.join(bringup_dir, "launch")
	pkg_home = get_package_share_directory("x3plus_nav2")
	pkg_localization = get_package_share_directory("x3plus_localization")
	pkg_bringup = get_package_share_directory("x3plus_bringup")
	pkg_wrapper = get_package_share_directory("x3plus_wrapper")
	pkg_sllidar_ros2 = get_package_share_directory("sllidar_ros2")
	# endregion

	# region  Create the launch configuration variables
	# args that can be set from the command line or a default will be used
	mode = LaunchConfiguration("mode")
	use_case = LaunchConfiguration("use_case")
	slam = LaunchConfiguration("slam")
	namespace = LaunchConfiguration("namespace")
	use_namespace = LaunchConfiguration("use_namespace")
	map_yaml_file = LaunchConfiguration("map")
	use_sim_time = LaunchConfiguration("use_sim_time")
	params_file = LaunchConfiguration("params_file")
	autostart = LaunchConfiguration("autostart")
	use_composition = LaunchConfiguration("use_composition")
	use_respawn = LaunchConfiguration("use_respawn")
	# endregion

	# region  Launch configuration variables specific to simulation
	use_nav2 = LaunchConfiguration("use_nav2")

	# endregion

	# region  Declare the launch arguments

	declared_arguments = [
			DeclareLaunchArgument(
				"mode",
				default_value="simulation",
				description="Launch mode: simulation, companion, hil, robot).",
			),
			DeclareLaunchArgument(
				"use_case",
				default_value="slam",
				description="Use case for the robot: drive, slam, explore, explore-lite, explore-roadmap, explore-frontier.",
			),
			DeclareLaunchArgument(
				"slam", default_value="True", description="Whether run a SLAM"
			),
			DeclareLaunchArgument(
				"use_sim_time",
				default_value="true",
				description="Use simulation (Gazebo) clock if true",
			),
			DeclareLaunchArgument(
				"params_file",
				default_value=os.path.join(
					pkg_home, "config", "nav2_params-MPPIController.yaml"
				),
				description="Full path to the ROS2 parameters file to use for all launched nodes",
			),
			DeclareLaunchArgument(
				"autostart",
				default_value="true",
				description="Automatically startup the nav2 stack",
			),
			DeclareLaunchArgument(
				"use_composition",
				default_value="True",
				description="Whether to use composed bringup",
			),
			DeclareLaunchArgument(
				"use_respawn",
				default_value="False",
				description="Whether to respawn if a node crashes. Applied when composition is disabled.",
			),
			DeclareLaunchArgument(
				"use_robot_state_pub",
				default_value="True",
				description="Whether to start the robot state publisher",
			),
			DeclareLaunchArgument(
				"use_nav2",
				default_value="False",
				description="Whether to start the x3plus logic",
			),
			DeclareLaunchArgument(
				"map",
				default_value=os.path.join(pkg_home, "maps", "map.yaml"),
				description="Full path to the ROS2 map file to use for navigation",
			),
	]
	# endregion

	# description="Launch mode: simulation, companion, hil, robot).",
	launch_actions = [
		OpaqueFunction(function=evaluate_xacro),

		OpaqueFunction(function=derive_configs),

		# Instead of using IncludeLaunchDescription, directly launch the sllidar_node with remappings:
		Node(
			package="sllidar_ros2",
			executable="sllidar_node",
			name="sllidar_node",
			output="screen",
			parameters=[
				{
					"channel_type": "serial",
					"serial_port": "/dev/rplidar",
					"serial_baudrate": 460800,
					"frame_id": "laser_link",
					"inverted": False,
					"angle_compensate": True,
					"scan_mode": "Standard",
				}
			],
			remappings=[("scan", "scan_raw")],
			condition=IfCondition(
				PythonExpression(["'", mode, "' in ['companion', 'robot']"])
			),
		),
		# Declare the launch arguments for the wrapper node
		Node(
			package="x3plus_wrapper",
			executable="Mecanum_driver_X3Plus",
			name="hardware_wrapper",
			output="screen",
			parameters=[os.path.join(pkg_wrapper, "config", "calibration.yaml")],
			condition=IfCondition(
				PythonExpression(["'", mode, "' in ['companion', 'robot']"])
			),
		),
		# region start logic for the robot
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				PathJoinSubstitution(
					[
						FindPackageShare("x3plus_bringup"),
						"launch",
						"journey.launch.py",
					]
				)
			),
			launch_arguments={
				"use_sim_time": LaunchConfiguration("use_sim_time"),
				"robot_name": LaunchConfiguration("robot_name", default="x3plus_bot"),
				"mode": LaunchConfiguration("mode"),
				"use_case": LaunchConfiguration("use_case"),
				# "visualize": LaunchConfiguration("visualize"),
				"use_nav2": LaunchConfiguration("use_nav2"),
				"map": LaunchConfiguration("map"),
				"params_file": LaunchConfiguration("params_file"),
				"slam": LaunchConfiguration("slam"),
				"autostart": LaunchConfiguration("autostart"),
				"use_composition": LaunchConfiguration("use_composition"),
				"use_respawn": LaunchConfiguration("use_respawn"),
			}.items(),
			condition=IfCondition(
				PythonExpression(["'", mode, "' in ['hil', 'robot']"])
			),
		),
		# endregion
	]

	return LaunchDescription(declared_arguments + launch_actions)
