#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
class Drone(Node):
    #COde came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
    def __init__(self):
        #Super constructor registers drone_controller with ros2 and lets me use its funcitonalliy 
        #This i sth enode name i see when I run ros2 list
        
        super().__init__('drone_hover')
        #These are services
        #Need a respone/acknowledgement
        self.arm_client = self.create_client(CommandBool,  '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode,  '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        #PoseStamped is a message type you publish to a topic
        #Just streaming dat acontiniously 
        self.position_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        #Get position from fcu
        self.position_subscriber = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.position_callback, qos)
        self.cur_pose = None
        self.state = State()
        self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

    
    
    def set_position(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.position_publisher.publish(pose)
    
    def position_callback(self, msg):
         self.cur_pose = msg
        # self.cur_position[0] = msg.pose.position.x
    
    def state_callback(self, msg):
        self.state = msg
    
    def arm(self):
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
    
    # def takeoff(self):
    #     req = CommandTOL.Request()
    #     req.value = True
    #     #Sends request
    #     future = self.takeoff_client.call_async(req)
    #     #Listens for request
    #     #Waits for request
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is not None:
    #         if future.result().success:
    #             self.get_logger().info('Takeoff successful')
    #             return True
    #         else:
    #             self.get_logger().warn('Failed to takeoff')
    #             return False
    #     else:
    #         self.get_logger().error('Failed to takeoff')
    #         return False
def main(args=None):
    #Starts ROS 2 engine
    rclpy.init(args=args)
    try:
        drone = Drone()
        # drone.get_logger().info('Setting mode to GUIDED')
        # if not drone.set_mode('GUIDED'):
        #     drone.get_logger().error('Failed to set GUIDED mode. Exiting...')
        #     return
        drone.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not drone.state.connected:
            rclpy.spin_once(drone, timeout_sec=0.1)

        drone.get_logger().info('FCU connected!')
        drone.get_logger().info('Arming the vehicle...')
        for _ in range(50):
            drone.set_position(0.0, 0.0, 3.0)
            rclpy.spin_once(drone, timeout_sec=0.05)
        if not drone.set_mode('GUIDED'):
            drone.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        if not drone.arm():
            drone.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        x_goal = 0.0
        y_goal = 0.0
        z_goal = 3.0
        while rclpy.ok():
            drone.set_position(x_goal, y_goal, z_goal)
            #callbacks like subscribers don't run automaitcally in background
            #only run when you give executor chance to process them
            
            rclpy.spin_once(drone, timeout_sec=0.05)
            #rclpy.spin(node) blocks forever, continuosuly processing until you control C or node shuts down
            #Used when node is purely reactive 
            z = drone.cur_pose.pose.position.z
            drone.get_logger().info(f'Current altitude: {z:.2f}m')
            if z >= z_goal-0.1:  # close enough to 5m
                drone.get_logger().info('Reached target altitude')
                break
    except KeyboardInterrupt:
        drone.get_logger().info('Flight interrupted by user')
    except Exception as e:
        drone.get_logger().error(f'An error occurred: {e}')
    finally:
        drone.disarm()
        drone.destroy_node()
        rclpy.shutdown()
    
    
 
if __name__ == '__main__':
    main()