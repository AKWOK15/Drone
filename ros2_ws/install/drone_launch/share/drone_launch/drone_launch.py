from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mavros_node =  Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[
                {
                    'fcu_url': 'udp://127.0.0.1:14550@'
                }
            ]
        )
    drone_hover_node = Node(
            package='autonomy_py',
            executable='drone_hover',
            name='autonomy_py'
        )
    
    return LaunchDescription([
        mavros_node,
        drone_hover_node
    ])