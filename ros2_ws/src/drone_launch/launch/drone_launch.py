from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mavros_node =  Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            {
                'fcu_url': 'udp://127.0.0.1:14550@',
                'plugin_denylist': ['param']
            }
        ]
    )
    drone_hover_node = Node(
        package='controls_py',
        executable='drone_hover',
        name='drone_hover_node'
    )
    
    track_person_node = Node(
        package='perception_py',
        executable='track_person',
        name='track_person_node'
    )
    
    return LaunchDescription([
        mavros_node,
        #drone_hover_node,
        track_person_node
    ])
