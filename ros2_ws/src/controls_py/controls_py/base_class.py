#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget
import time
from std_msgs.msg import Int8

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
class BaseDroneNode(Node):
	#COde came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
	def __init__(self, name):
		#Super constructor registers node with ros2 and lets me use its funcitonalliy 
		
		super().__init__(name)
		self.PERSON_POSITION_TIMEOUT_SEC = 1.5
		self.target = None
		self.enu_cur_position = None
		self.state = State()
		self.home = None
		self.fps = 0
		self.required_fps = 3
		# Watchdog for person_position message
		self.last_person_position_time = None  # None means we haven't received any yet
		self.rtl_triggered = False

		#These are services
		#Need a respone/acknowledgement
		self.arm_client = self.create_client(CommandBool,  '/mavros/cmd/arming')
		self.mode_client = self.create_client(SetMode,	'/mavros/set_mode')
		self.takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

		# forward, left, up
		self.flu_position_publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
	
		# north, east, up
		self.enu_position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 20)	
		self.enu_position_subscriber = self.create_subscription(
			PoseStamped,
			'/mavros/local_position/pose',
			self.enu_position_callback,
			qos
		)
		self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
		self.person_position_subscriber = self.create_subscription(Point, 'person_position', self.person_position_callback, 10)
		self.fps_subscriber = self.create_subscription(Int8, 'fps', self.fps_callback, 10)
		self.watchdog_timer = self.create_timer(0.5, self.person_position_watchdog)

	# -------------------------------------------------------------------------
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

		if elapsed > self.PERSON_POSITION_TIMEOUT_SEC:
			self.get_logger().warn(
				f'No person_position for {elapsed:.1f}s — camera lost target. Triggering RTL.'
			)
			self.rtl_triggered = True
			self._execute_rtl_or_land()

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

	def enu_position_callback(self, msg):
		# PoseStamped: position lives at msg.pose.position.x/y/z
		self.enu_cur_position = msg

	def state_callback(self, msg):
		self.state = msg

	def fps_callback(self, msg):
		self.fps = msg.data        
	
	def arm(self):
		self.get_logger().info('Arm started')
		while not self.arm_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for arming service...')
		req = CommandBool.Request()
		req.value = True
		#Sends request
		future = self.arm_client.call_async(req)
		#Listens for request
		#Waits for request
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			if future.result().success:
				self.get_logger().info('Vehicle armed successfully')
				return True
			else:
				self.get_logger().warn(f'Failed to arm vehicle: success={future.result().success}, result={future.result().result}')
				self.get_logger().warn('Failed to arm vehicle')
				return False
		else:
			self.get_logger().error('Arming service call failed')
			return False
		
	def disarm(self) -> bool:
		"""Disarm the quadcopter."""
		req = CommandBool.Request()
		req.value = False

		future = self.arm_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		if future.result() is not None:
			if future.result().success:
				self.get_logger().info('Vehicle disarmed successfully')
				return True
			else:
				self.get_logger().warn(f'Failed to arm vehicle: success={future.result().success}, result={future.result().result}')
				self.get_logger().warn('Failed to disarm vehicle')
				return False
		else:
			self.get_logger().error('Disarming service call failed')
			return False
	
	def set_mode(self, mode):
		"""
		Set flight mode.

		Common modes: STABILIZED, GUIDED, RTL, LAND
		"""
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
				self.get_logger().warn(f'Failed to set mode to {mode}')
				self.get_logger().warn(f'Failed to set mode: mode_sent={future.result().mode_sent}')
				return False
		else:
			self.get_logger().error('Set mode service call failed')
			return False

	def set_home(self, msg):
		# Stores full PoseStamped so ned_move can use it directly
		self.home = msg
		self.get_logger().info(f'Set home: {msg}')

	def set_flu_position(self, forward, left, up):
		msg = PositionTarget()
		msg.header.frame_id = "body_frame"
		msg.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
		# Use position
		msg.type_mask = 0b111111111000
		msg.position.x = forward
		msg.position.y = left
		msg.position.z = up
		
		msg.header.stamp = self.get_clock().now().to_msg()
		self.flu_position_publisher.publish(msg)
	
	def set_enu_position(self, goal_pose):
		self.enu_position_publisher.publish(goal_pose)
	
	def flu_move(self, duration):
		forward = self.get_parameter('forward').get_parameter_value().double_value
		left = self.get_parameter('left').get_parameter_value().double_value
		up = self.get_parameter('up').get_parameter_value().double_value
		self.get_logger().info(f'forward param: {forward}')
		self.get_logger().info(f'left param: {left}')
		self.get_logger().info(f'right param: {up}')
		self.get_logger().info('Started flu_move')
		cur_time = time.time()
		while time.time() - cur_time < duration:
			self.set_flu_position(forward, left, up) 
			self.get_logger().info(f'enu x: {self.enu_cur_position.pose.position.x}')
			self.get_logger().info(f'enu y: {self.enu_cur_position.pose.position.y}')
			self.get_logger().info(f'enu z: {self.enu_cur_position.pose.position.z}')
			rclpy.spin_once(self, timeout_sec=0.05)
		self.get_logger().info('Finished flu_move')
		return True
	
	# east, north, up
	def enu_move(self, goal_pose):
		self.get_logger().info('Started enu_move')
		while (
			abs(self.enu_cur_position.pose.position.x - goal_pose.pose.position.x) > 0.4 or
			abs(self.enu_cur_position.pose.position.y - goal_pose.pose.position.y) > 0.4 or
			abs(self.enu_cur_position.pose.position.z - goal_pose.pose.position.z) > 0.2
		):
			self.set_enu_position(goal_pose)
			rclpy.spin_once(self, timeout_sec=0.05)
		self.get_logger().info('Finished enu_move')
		
	def takeoff(self, altitude):
		req = CommandTOL.Request()
		req.altitude = altitude
		# req.min_pitch = 0.0
		# req.yaw = 0.0
		# req.latitude = 0.0
		# req.longitude = 0.0
		#Sends request
		future = self.takeoff_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		
		duration = 5
		if future.result() is not None:
			self.get_logger().info(f'takeoff result: {future.result()}')
			if future.result().success:
				start = time.time()
				self.get_logger().info(f'Taking off to altitude: {altitude}')
				while time.time() - start < duration:	
					self.get_logger().info(f'enu z: {self.enu_cur_position.pose.position.z:.2f}')
					rclpy.spin_once(self, timeout_sec=0.05)

				#while self.enu_cur_position.pose.position.z < 0.95 * altitude:
				#	self.get_logger().info(f'enu z: {self.enu_cur_position.pose.position.z:.2f}')
				#	rclpy.spin_once(self, timeout_sec=0.05)
				self.get_logger().info(f'Finished takeoff')
				return True
			else:
				self.get_logger().warn('Failed to takeoff')
				return False
		else:
			self.get_logger().error('Failed to takeoff')
			return False

	def rtl_or_land(self):
		if self.set_mode('RTL'):
			self.get_logger().info('RTL engaged')
			while self.state.armed:
				self.get_logger().info(f'enu z: {self.enu_cur_position.pose.position.z:.2f}')
				rclpy.spin_once(self, timeout_sec=0.1)
			self.get_logger().info('Vehicle disarmed, RTL complete')
		elif self.set_mode('LAND'):
			self.get_logger().warn('RTL failed. LAND engaged.')
		else:
			self.get_logger().error('Both RTL and LAND failed. Flying home manually.')
			self.ned_move(self.home)
			self.get_logger().info('Reached home.')

