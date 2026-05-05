#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
import time

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
from controls_py.base_class import BaseDroneNode
class HoverNode(BaseDroneNode):
	def __init__(self):
		super().__init__('hover_node')
		self.declare_parameter('takeoff_height', 0.5)

def main(args=None):
	rclpy.init(args=args)
	drone = None
	try:
		drone = HoverNode()
		 
		# Wait for FCU connection
		drone.get_logger().info('Waiting for FCU connection...')
		while rclpy.ok() and not drone.state.connected:
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('FCU connected!')
		
		# Wait for first position message from FCU
		while drone.enu_cur_position is None:	
			rclpy.spin_once(drone, timeout_sec=0.1)	
		drone.set_home(drone.enu_cur_position)	
		# Pre-arm: stream setpoints before requesting mode/arm
		for _ in range(50):
			drone.get_logger().info('Setting flu position')
			drone.set_flu_position(0.0, 0.0, 0.0)
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
		drone.get_logger().info(f'enu z: {drone.enu_cur_position.pose.position.z:.2f}')
		# Arm
		if not drone.arm():
			return
		drone.get_logger().info(f'enu z: {drone.enu_cur_position.pose.position.z:.2f}')
		# Wait for FC to actually be armed
		while not drone.state.armed:
			drone.get_logger().info('Waiting for vehicle to arm...')
			rclpy.spin_once(drone, timeout_sec=0.1)

		drone.get_logger().info('Armed! Sending takeoff...')
		drone.home = drone.enu_cur_position
		takeoff_height = drone.get_parameter('takeoff_height').get_parameter_value().double_value 
		drone.get_logger().info(f'takeoff_height: {takeoff_height}')
		drone.get_logger().info(f'enu z: {drone.enu_cur_position.pose.position.z:.2f}')
		# Takeoff
		if not drone.takeoff(takeoff_height):
			return

		while rclpy.ok():
			goal_pose = PoseStamped()
			goal_pose.pose.position.x = drone.home.pose.position.x + 10
			goal_pose.pose.position.y = drone.home.pose.position.y	
			goal_pose.pose.position.z = drone.home.pose.position.z
			drone.enu_move(goal_pose)		
			drone.rtl_or_land()

	except KeyboardInterrupt:
		drone.get_logger().info('Flight interrupted by user')
	except Exception as e:
		drone.get_logger().error(f'An error occurred: {e}')
	finally:
		if drone is not None:
			drone.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
