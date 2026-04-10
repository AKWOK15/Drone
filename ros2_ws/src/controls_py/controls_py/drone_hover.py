#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State

#Need to add the autonomy_py. because after colcon build, scripts get installed to:
#install/autonomy_py/lib/python3.x/site-packages/autonomy_py/
#sys.path sees only autonomy_py, so drone_class can't be found, need autonomy_py.drone_class
#sys.path is basically a var that determines where on the file system Python will look for module to import 
#when you run import, python will search directory and then every other direcoty in sys.path 
from controls_py.drone_class import Drone
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def main(args=None):
    #Starts ROS 2 engine
    rclpy.init(args=args)
    try:
        drone = Drone()
        drone.get_logger().info('Waiting for FCU connection...')
        while rclpy.ok() and not drone.state.connected:
            rclpy.spin_once(drone, timeout_sec=0.1)

        drone.get_logger().info('FCU connected!')
        drone.get_logger().info('Arming the vehicle...')
        for _ in range(50):
            drone.set_position(0.0, 0.0, 3.0)
            rclpy.spin_once(drone, timeout_sec=0.05)
        drone.get_logger().info('Setting mode to GUIDED')
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