#!/usr/bin/env python3
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
import time
from std.msgs.msg import Int8

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# How many seconds without a person_position message before RTL is triggered
PERSON_POSITION_TIMEOUT_SEC = 1.5


class Drone(Node):
	# Code came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
	def __init__(self):
		super().__init__('intercept')
		self.declare_parameter('height_goal', 1.0)
		self.declare_parameter('takeoff_height', 0.5)
		self.target = None
		self.cur_position = None
		self.state = State()
		self.home = None
		self.fps = 0
		self.required_fps = 4
		# Watchdog for person_position message
		self.last_person_position_time = None  # None means we haven't received any yet
		self.rtl_triggered = False

		# Services
		self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
		self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
		self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

		# Publishers
		self.position_publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

		# Subscribers
		self.position_subscriber = self.create_subscription(
			PoseStamped,
			'/mavros/local_position/pose',
			self.position_callback,
			qos
		)
		self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
		self.person_position_subscriber = self.create_subscription(Point, 'person_position', self.person_position_callback, 10)
		self.fps_subscriber = self.create_subscription(Int8, 'fps', self.fps_callback, 10)
		
		# Watchdog timer: fires every second, checks how long since last person_position
		self.watchdog_timer = self.create_timer(0.5, self.person_position_watchdog)


 -------------------------------------------------------------------------
	# Watchdog
	# -------------------------------------------------------------------------

	def person_position_watchdog(self):
		"""Trigger RTL if person_position topic goes silent during flight."""
		if not self.state.armed or self.rtl_triggered:
			return

		# If we never received a detection at all, nothing to watch yet
		if self.last_person_position_time is None:
			return

		elapsed = (self.get_clock().now() - self.last_person_position_time).nanoseconds / 1e9

		if elapsed > PERSON_POSITION_TIMEOUT_SEC:
			self.get_logger().warn(
				f'No person_position for {elapsed:.1f}s — camera lost target. Triggering RTL.'
			)
			self.rtl_triggered = True
			self._execute_rtl_or_land()

	def _execute_rtl_or_land(self):
		if self.set_mode('RTL'):
			self.get_logger().info('RTL engaged from watchdog.')
		elif self.set_mode('LAND'):
			self.get_logger().warn('RTL failed from watchdog. LAND engaged.')
		else:
			self.get_logger().error('Both RTL and LAND failed from watchdog. Attempting manual land.')
			while abs(self.cur_position.pose.position.z - self.home.position.z) > 0.2:
				self.set_position(True)
				rclpy.spin_once(self, timeout_sec=0.05)
			self.get_logger().info('Landed')

	# -------------------------------------------------------------------------
	# Callbacks
	# -------------------------------------------------------------------------

	def person_position_callback(self, msg):
		self.target = msg
		# Reset watchdog timestamp on every received message
		self.last_person_position_time = self.get_clock().now()
		# If RTL was triggered but detections resumed, allow recovery
		if self.rtl_triggered:
			self.get_logger().info('person_position resumed — clearing RTL flag.')
			self.rtl_triggered = False

	def position_callback(self, msg):
		self.cur_position = msg

	def state_callback(self, msg):
		self.state = msg

	def fps_callback(self, msg):
		self.fps = msg.data
	# -------------------------------------------------------------------------
	# Flight commands
	# -------------------------------------------------------------------------

	# Set home in case it needs to manually land
	def set_home(self, msg):
		self.home = msg
		self.get_logger().info(f'Set home: {msg}')

	def set_position(self, home=False):
		msg = PositionTarget()
		msg.header.frame_id = "home"
		msg.coordinate_frame = 9
		msg.type_mask = 0b110111111000
		if home:
			msg.position.x = self.home.pose.position.x
			msg.position.y = self.home.pose.position.y
			msg.position.z = self.home.pose.position.z
		else:
			msg.position.x = self.target.x
			msg.position.y = self.target.y
			msg.position.z = self.target.z
		msg.header.stamp = self.get_clock().now().to_msg()
		self.position_publisher.publish(msg)

	def arm(self):
		self.get_logger().info('Arm started')
		while not self.arm_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for arming service...')
		req = CommandBool.Request()
		req.value = True
		future = self.arm_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			if future.result().success:
				self.get_logger().info('Vehicle armed successfully')
				return True
			else:
				self.get_logger().warn(f'Failed to arm vehicle: success={future.result().success}, result={future.result().result}')
				return False
		else:
			self.get_logger().error('Arming service call failed')
			return False

	def disarm(self):
		req = CommandBool.Request()
		req.value = False
		future = self.arm_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		if future.result() is not None:
			if future.result().success:
				self.get_logger().info('Vehicle disarmed successfully')
				return True
			else:
				self.get_logger().warn(f'Failed to disarm vehicle: success={future.result().success}, result={future.result().result}')
				return False
		else:
			self.get_logger().error('Disarming service call failed')
			return False

	def set_mode(self, mode):
		"""Set flight mode. Common modes: STABILIZED, GUIDED, RTL, LAND"""
		req = SetMode.Request()
		req.base_mode = 0
		req.custom_mode = mode
		future = self.mode_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			if future.result().mode_sent:
				self.get_logger().info(f'Mode set to {mode}')
				return True
			else:
				self.get_logger().warn(f'Failed to set mode to {mode}: mode_sent={future.result().mode_sent}')
				return False
		else:
			self.get_logger().error('Set mode service call failed')
			return False

	def hold_position(self, duration):
		self.get_logger().info('Started hold_position')
		cur_time = time.time()
		while time.time() - cur_time < duration:
			self.set_position()
			rclpy.spin_once(self, timeout_sec=0.05)
		self.get_logger().info('Finished hold_position')

	def takeoff(self, altitude):
		req = CommandTOL.Request()
		req.altitude = altitude
		req.min_pitch = 0.0
		req.yaw = 0.0
		req.latitude = 0.0
		req.longitude = 0.0
		future = self.takeoff_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			self.get_logger().info(f'Takeoff result: {future.result()}')
			if future.result().success:
				while self.cur_position.pose.position.z < 0.95 * altitude:
					self.get_logger().info(f'Taking off, current altitude: {self.cur_position.pose.position.z}')
					rclpy.spin_once(self, timeout_sec=0.05)
				return True
			else:
				self.get_logger().warn('Failed to takeoff')
				return False
		else:
			self.get_logger().error('Failed to takeoff')
			return False


def main(args=None):
	rclpy.init(args=args)
	drone = None
	try:
		drone = Drone()
		height_goal = drone.get_parameter('height_goal').get_parameter_value().double_value

		# Wait for FCU connection
		drone.get_logger().info('Waiting for FCU connection...')
		while rclpy.ok() and not drone.state.connected:
			rclpy.spin_once(drone, timeout_sec=0.1)
		drone.get_logger().info('FCU connected!')

		while drone.cur_position is None:
			drone.get_logger().info('Waiting for drone position...')
			rclpy.spin_once(drone, timeout_sec=0.1)

		drone.set_home(drone.cur_position)

		# Pre-arm: stream setpoints before requesting mode/arm
		for _ in range(50):
			drone.set_position(True)
			# don't want timeout to be too low because if ROS executor is busy during short spin, setpoint stream could dropbs
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

		drone.get_logger().info('Target acquired. Proceeding to arm.')
		
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
		z_start = drone.cur_position.pose.position.z
		takeoff_height = drone.get_parameter('takeoff_height').get_parameter_value().double_value + z_start
		if not drone.takeoff(takeoff_height):
			return

		# Main flight loop — watchdog runs automatically via timer
		while rclpy.ok():
			while not drone.rtl_triggered:
				drone.set_position()
				rclpy.spin_once(drone, timeout_sec=0.05)
			# If watchdog triggered RTL, let the FC handle it and exit the loop
			if drone.rtl_triggered:
				drone.get_logger().info('No new messages from track_person. Moving to RTL')
				break

			rclpy.spin_once(drone, timeout_sec=0.05)

	except KeyboardInterrupt:
		drone.get_logger().info('Flight interrupted by user')
		if drone is not None and drone.state.armed:
			drone.set_mode('RTL')
	except Exception as e:
		drone.get_logger().error(f'An error occurred: {e}')
	finally:
		if drone is not None:
			drone.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
