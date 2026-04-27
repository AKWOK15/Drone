#!/usr/bin/env python2
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

# Need to add the autonomy_py. because after colcon build, scripts get installed to:
# install/autonomy_py/lib/python3.x/site-packages/autonomy_py/
# sys.path sees only autonomy_py, so drone_class can't be found, need autonomy_py.drone_class
# sys.path is basically a var that determines where on the file system Python will look for module to import
# when you run import, python will search directory and then every other directory in sys.path
from controls_py.drone_class import Drone

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def main(args=None):
	# Starts ROS 2 engine
	rclpy.init(args=args)

	try:
		# Give drone class the same name as name specified in launch file to pass parameter
		drone = Drone(node_name='drone_hover')
		height_goal = drone.get_parameter('height_goal').get_parameter_value().double_value
		drone.get_logger().info(f'height_goal: {height_goal}')
		# Take off one meter above where we currently are
		# Wait for FCU connection
		drone.get_logger().info('Waiting for FCU connection...')
		while rclpy.ok() and not drone.state.connected:
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('FCU connected!')
		drone.get_logger().info(f'drone_get_position: {drone.get_position()}')	
		while drone.get_position() == None:	
			drone.get_logger().info('Waiting for drone position')	
			rclpy.spin_once(drone, timeout_sec=0.1)	
		takeoff_height = drone.get_position().pose.position.z + 0.5
		
		# Pre-arm: stream setpoints before requesting mode/arm
		drone.get_logger().info('Arming the vehicle...')
		for _ in range(50):
			drone.set_position(drone.get_position().pose.position.x, drone.get_position().pose.position.y, drone.get_position().pose.position.z)
			rclpy.spin_once(drone, timeout_sec=0.05)
		
		
		# After set_mode call, wait for GUIDED confirmation
		drone.get_logger().info('Setting mode to GUIDED')
		if not drone.set_mode('GUIDED'):
			drone.get_logger().error('Failed to set GUIDED mode. Exiting...')
			return

		# Wait for FC to actually be in GUIDED
		while drone.state.mode != 'GUIDED':
			drone.get_logger().info(f'Waiting for GUIDED mode, current: {drone.state.mode}')
			rclpy.spin_once(drone, timeout_sec=0.1)

		# Arm
		if not drone.arm():
			drone.get_logger().error('Failed to arm vehicle. Exiting...')
			return

		# Wait for FC to actually be armed
		while not drone.state.armed:
			drone.get_logger().info('Waiting for vehicle to arm...')
			rclpy.spin_once(drone, timeout_sec=0.1)

		drone.get_logger().info('Armed! Sending takeoff...')

		# Takeoff
		if not drone.takeoff(takeoff_height):
			drone.get_logger().error('Failed to takeoff. Exiting...')
			return
	
		# Flight loop
		x_goal = drone.get_position().pose.position.x 
		y_goal = drone.get_position().pose.position.y
		z_goal = drone.get_position().pose.position.z + height_goal

		while rclpy.ok():
			#Getting to proper altitude for takeoff
			while drone.get_position().pose.position.z < 0.95 * takeoff_height:
				drone.get_logger().info(f'Taking off to proper altitude, cur altitude: {drone.get_position().pose.position.z}')
				rclpy.spin_once(drone, timeout_sec=0.1)
			drone.go_to_position(x_goal, y_goal, z_goal)
			drone.hold_position(x_goal, y_goal, z_goal, 15)

			if not drone.set_mode('LAND'):
				drone.get_logger().error('Failed to set LAND. Attempting to autonomously land')
				x_goal = 0.0
				y_goal = 0.0
				z_goal = 0.0
				drone.go_to_position(x_goal, y_goal, z_goal)
				drone.disarm()
				return

	except KeyboardInterrupt:
		drone.get_logger().info('Flight interrupted by user')
	except Exception as e:
		drone.get_logger().error(f'An error occurred: {e}')
	finally:
		drone.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
