#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode

class Drone(Node):
    #COde came from https://github.com/mavlink/mavros/blob/ros2/mavros_examples/mavros_examples/flight_drone.py
    def __init__(self):
        #Super constructor registers drone_controller with ros2 and lets me use its funcitonalliy 
        
        super().__init__('drone_controller')
        self.arm_client = self.create_client(CommandBool,  '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode,  '/mavros/set_mode')
    
    def arm(self):
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
        req.custom_mode = mode

        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().warn(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Set mode service call failed')
            return False
        
def main(args=None):
    #Starts ROS 2 engine
    rclpy.init(args=args)
    try:
        drone = Drone()
        drone.get_logger().info('Setting mode to GUIDED')
        if not drone.set_mode('GUIDED'):
            drone.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        drone.get_logger().info('Arming the vehicle...')
        if not drone.arm():
            drone.get_logger().error('Failed to arm vehicle. Exiting...')
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