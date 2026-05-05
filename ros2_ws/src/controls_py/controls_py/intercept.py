#!/usr/bin/env python3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
import time
from std_msgs.msg import Int8
from controls_py.base_class import BaseDroneNode

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class InterceptNode(BaseDroneNode):
	# Code came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
	def __init__(self):
		super().__init__('intercept_node')
		self.declare_parameter('takeoff_height', 0.5)


def main(args=None):
	rclpy.init(args=args)
	drone = None
	try:
		drone = InterceptNode()

		# Wait for FCU connection
		drone.get_logger().info('Waiting for FCU connection...')
		while rclpy.ok() and not drone.state.connected:
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('FCU connected!')

		while drone.enu_cur_position is None:
			drone.get_logger().info('Waiting for drone position...')
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.set_home(drone.enu_cur_position)

		# Pre-arm: stream setpoints before requesting mode/arm
		for _ in range(50):
			drone.set_flu_position(0.0, 0.0, 0.0)
			rclpy.spin_once(drone, timeout_sec=0.05)

		drone.get_logger().info('Setting mode to GUIDED')
		if not drone.set_mode('GUIDED'):
			drone.get_logger().error('Failed to set GUIDED mode. Exiting...')
			return

		while drone.state.mode != 'GUIDED':
			drone.get_logger().info(f'Waiting for GUIDED mode, current: {drone.state.mode}')
			rclpy.spin_once(drone, timeout_sec=0.1)

		# Wait for first detection before arming
		drone.get_logger().info('Waiting for initial person_position detection...')
		while drone.target is None:
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('Target acquired. Proceeding to wait for FPS.')

		drone.get_logger().info(f'Waiting for {drone.required_fps} fps')
		while drone.fps < drone.required_fps:
			drone.get_logger().info(f'fps: {drone.fps}')
			rclpy.spin_once(drone, timeout_sec=0.1)

		if not drone.arm():
			return

		while not drone.state.armed:
			drone.get_logger().info('Waiting for vehicle to arm...')
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('Armed! Sending takeoff...')

		takeoff_height = drone.get_parameter('takeoff_height').get_parameter_value().double_value
		if not drone.takeoff(takeoff_height):
			return

		while rclpy.ok() and not drone.rtl_triggered:
			if drone.target is not None:
				x = drone.target.x
				y = drone.target.y
				z = drone.target.z
			else:
				x, y, z = 0.0, 0.0, 0.0
			drone.set_flu_position(x, y, z)
			rclpy.spin_once(drone, timeout_sec=0.05)

		if drone.rtl_triggered:
			drone.get_logger().info('RTL triggered — exiting flight loop.')

	except KeyboardInterrupt:
		if drone is not None:
			drone.get_logger().info('Flight interrupted by user')
			if drone.state.armed:
				drone.set_mode('RTL')
	except Exception as e:
		if drone is not None:
			drone.get_logger().error(f'An error occurred: {e}')
		else:
			print(f'An error occurred before node init: {e}')
	finally:
		if drone is not None:
			drone.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()