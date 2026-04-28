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

class Drone(Node):
	#COde came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
	def __init__(self):
		#Super constructor registers node with ros2 and lets me use its funcitonalliy 
		
		super().__init__('hover')
		self.declare_parameter('height_goal', 1.0)
		self.declare_parameter('takeoff_height', 0.5)
		#These are services
		#Need a respone/acknowledgement
		self.arm_client = self.create_client(CommandBool,  '/mavros/cmd/arming')
		self.mode_client = self.create_client(SetMode,	'/mavros/set_mode')
		self.takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
		#PoseStamped is a message type you publish to a topic
		#Just streaming dat acontiniously 
		self.position_publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
		
		#Get position from fcu
		self.position_subscriber = self.create_subscription(PositionTarget, '/mavros/setpoint_raw/target_local', self.position_callback, qos)
		self.cur_position = None
		self.state = State()
		self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
	   
	def set_position(self, x, y, z):
		msg = PositionTarget()
		msg.header.frame_id = "home"
		msg.coordinate_frame = 9
		msg.type_mask = 0b110111111000
		msg.position.x = x
		msg.position.y = y
		msg.position.z = z 
		msg.header.stamp = self.get_clock().now().to_msg()
		self.position_publisher.publish(msg)
	
	def position_callback(self, msg):
		self.cur_position = msg
	
	def state_callback(self, msg):
		self.state = msg
	
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

	#duration is in seconds
	def hold_position(self, x_goal, y_goal, z_goal, duration):
		self.get_logger().info('Started hold_position')
		self.get_logger().info(f'Frame coordinate frame: {self.cur_position.coordinate_frame}')
		cur_time = time.time()
		while time.time() - cur_time < duration:
			self.set_position(x_goal, y_goal, z_goal)
			rclpy.spin_once(self, timeout_sec=0.05)													
			# z = self.cur_position.position.z
			# self.get_logger().info(f'Current altitude: {z:.2f}m') 
		self.get_logger().info('Finished hold_position')

	def go_to_position(self, x_goal, y_goal, z_goal):
		z = self.cur_position.position.z
		self.get_logger().info('Started go_to_position')
		while abs(z - z_goal) > 0.1:
			self.set_position(x_goal, y_goal, z_goal)
			rclpy.spin_once(self, timeout_sec=0.05)
			z = self.cur_position.position.z
			# x = self.cur_position.position.x			
			# y = self.cur_position.position.y
			# z = self.cur_position.position.z
			# self.get_logger().info(f'Current x: {x:.2f}m') 
			# self.get_logger().info(f'Current y: {y:.2f}m') 
			# self.get_logger().info(f'Current z:: {z:.2f}m') 
		self.get_logger().info('Finished go_to_position')
		return True

	def takeoff(self, altitude):
		req = CommandTOL.Request()
		req.altitude = altitude
		req.min_pitch = 0.0
		req.yaw = 0.0
		req.latitude = 0.0
		req.longitude = 0.0
		#Sends request
		future = self.takeoff_client.call_async(req)
		#Listens for request
		#Waits for request
		rclpy.spin_until_future_complete(self, future)

		if future.result() is not None:
			self.get_logger().info(f'takeoff result: {future.result()}')
			if future.result().success:
				while self.cur_position.position.z < 0.95 * altitude:
					self.get_logger().info(f'Taking off to proper altitude, cur altitude: {drone.cur_position.position.z}')
					rclpy.spin_once(self, timeout_sec=0.05)
				return True
			else:
				self.get_logger().warn('Failed to takeoff')
				return False
		else:
			self.get_logger().error('Failed to takeoff')
			return False

# Need to add the autonomy_py. because after colcon build, scripts get installed to:
# install/autonomy_py/lib/python3.x/site-packages/autonomy_py/
# sys.path sees only autonomy_py, so drone_class can't be found, need autonomy_py.drone_class
# sys.path is basically a var that determines where on the file system Python will look for module to import
# when you run import, python will search directory and then every other directory in sys.path



def main(args=None):
	# Starts ROS 2 engine
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
		while drone.cur_position == None:	
			drone.get_logger().info('Waiting for drone position')	
			rclpy.spin_once(drone, timeout_sec=0.1)	
		
		# Pre-arm: stream setpoints before requesting mode/arm
		
		for _ in range(50):
			drone.set_position(drone.cur_position.position.x, drone.cur_position.position.y, drone.cur_position.position.z)
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
			return

		# Wait for FC to actually be armed
		while not drone.state.armed:
			drone.get_logger().info('Waiting for vehicle to arm...')
			rclpy.spin_once(drone, timeout_sec=0.1)

		drone.get_logger().info('Armed! Sending takeoff...')
		z_start = drone.cur_position.position.z
		takeoff_height = drone.get_parameter('takeoff_height').get_parameter_value().double_value + z_start 
		# Takeoff
		if not drone.takeoff(takeoff_height):
			return
	
		while rclpy.ok():
			#Getting to proper altitude for takeoff
			x_goal = self.cur_position.position.x				
			y_goal = self.cur_position.position.y
			z_goal = self.cur_position.position.z + height_goal 
			drone.go_to_position(x_goal, y_goal, z_goal)
			drone.hold_position(x_goal, y_goal, z_goal, 5)
			if drone.set_mode('RTL'):
				drone.get_logger().info('RTL engaged, letting FC handle return')
				break
			elif drone.set_mode('LAND'):
				drone.get_logger().info('RTL failed. Land engaged. Attempting to autonomously land')
				break
			else:
				drone.get_logger.info('Land failed. Manually land')
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
