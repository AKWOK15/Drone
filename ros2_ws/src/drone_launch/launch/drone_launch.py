from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration

# Use DeclareLaunchArgument to expose argument outside of launch file, so I can pass in arguments from command line
launch_args = [
	DeclareLaunchArgument(
		'control_node_param',
		default_value='takeoff_rtl_node',
		description='Which control node to run: takeoff_rtl_node, intercept_node'
	),
	DeclareLaunchArgument(
		'data_subfolder_param',
		default_value='takeoff_rtl_node_1_05-06_15:50',
		description='Which data subfolder to send data to'
	),
]

def launch_setup(context):
	# .perform(context) extracts actual string value at runtime
	# LaunchConfiguration is local to launch file and packages argument value that gets unpacked at runtime and passed to Node
	# ROS2 is designed this way because arg values might not be known at Python parse time, could come from CLI so it's lazy and only unpacks at last second 
	control_node = LaunchConfiguration('control_node_param').perform(context)
	data_subfolder = LaunchConfiguration('data_subfolder_param').perform(context)

	print(f"[launch_setup] control_node: {control_node}")
	print(f"[launch_setup] data_subfolder: {data_subfolder}")

	mavros_node = Node(
		package='mavros',
		executable='mavros_node',
		parameters=[{
			'fcu_url': 'udp://127.0.0.1:14550@',
			'plugin_denylist': ['param']
		}]
	)

	takeoff_rtl_node = Node(
		package='controls_py',
		executable='takeoff_rtl_node',
		parameters=[{'takeoff_height': 1.2}]
	)

	intercept_node = Node(
		package='controls_py',
		executable='intercept_node',
		parameters=[{'takeoff_height': 0.6}]
	)

	track_person_node = Node(
		package='perception_py',
		executable='track_person',
		parameters=[{'data_subfolder': data_subfolder}]
	)

	nodes = [mavros_node, track_person_node]

	# Normal Python string comparison now works
	if control_node == 'intercept_node':
		nodes.append(intercept_node)
		print("[launch_setup] >>> Appending intercept_node")
	else:
		nodes.append(takeoff_rtl_node)
		print("[launch_setup] >>> Appending takeoff_rtl_node")

	return nodes

def generate_launch_description():
	# Need to add args to LaunchDescription to register it with the launch system or else the CLI won't recgonize my params
	ld = LaunchDescription(launch_args)
	# ROS2 launch creates and manages a LaunchContext object, which gets automically passes in as arg to python function that OpaqueFunction executes
	# Context holds all the resolved argument values
	ld.add_action(OpaqueFunction(function=launch_setup))
	return ld
