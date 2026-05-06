from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration

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
        parameters=[{'takeoff_height': 1.5}]
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
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
