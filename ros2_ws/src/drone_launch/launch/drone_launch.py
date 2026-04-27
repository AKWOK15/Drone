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
	#When I call drone class from within hover node, ROS2 will look up parameters registered to that node name
    drone_hover_node = Node(
        package='controls_py',
        executable='drone_hover',
		parameters=[
			{
				'height_goal': 1.0
			}
		]
    )
    
    track_person_node = Node(
        package='perception_py',
        executable='track_person',
    )
    
    return LaunchDescription([
        mavros_node,
        drone_hover_node,
        #track_person_node
    ])
