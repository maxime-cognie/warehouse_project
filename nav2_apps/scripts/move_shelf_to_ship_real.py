#!/usr/bin/env python3

import time
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from std_msgs.msg import String
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from typing import NamedTuple

class Goal(NamedTuple):
    name: str
    position: list
    orientation: list

# Initial position of the robot
initial_position = Goal(
    'initial_position',
    [-0.648, -0.662, 0.0],
    [0.0, 0.0, 0.306, 0.952])

# Shelf positions for picking
shelf_position = Goal(
    'shelf_position',
    [4.0, 0.33, 0.0],
    [0.0, 0.0, -0.4814, 0.8764])

intermediate_position = Goal(
    'intermediate_position',
    [1.0, 0.43, 0.0],
    [0.0, 0.0, 0.8680, 0.4966])

# Shipping destination for picked products
shipping_position = Goal(
    'shipping_position',
    [1.0, 1.3, 0.0],
    [0.0, 0.0, 0.8680, 0.4966])


# footprint of the robot
robot_fp = [
    Point32(x=0.307, y=0.0, z=0.0),
    Point32(x=0.284, y=0.125, z=0.0),
    Point32(x=0.219, y=0.222, z=0.0),
    Point32(x=0.121, y=0.287, z=0.0),
    Point32(x=0.006, y=0.310, z=0.0),
    Point32(x=-0.12, y=0.286, z=0.0),
    Point32(x=-0.22, y=0.221, z=0.0),
    Point32(x=-0.28, y=0.124, z=0.0),
    Point32(x=-0.31, y=0.0, z=0.0),
    Point32(x=-0.28, y=-0.125, z=0.0),
    Point32(x=-0.22, y=-0.222, z=0.0),
    Point32(x=-0.12, y=-0.287, z=0.0),
    Point32(x=-0.01, y=-0.310, z=0.0),
    Point32(x=0.122, y=-0.286, z=0.0),
    Point32(x=0.220, y=-0.221, z=0.0),
    Point32(x=0.284, y=0.124, z=0.0)
]

# footprint of the robot when lifting the shelf
robot_shelf_fp = [
    Point32(x=0.45, y=0.45, z=0.0), 
    Point32(x=0.45, y=-0.45, z=0.0), 
    Point32(x=-0.45, y=-0.45, z=0.0), 
    Point32(x=-0.45, y=0.45, z=0.0)
]

class ApproachShelf(Node):
    def __init__(self):
        super().__init__('approach_shelf_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
       

    # move the robot for mv_time
    # move the robot forward: direction = 1, backward: direction = -1
    def move_robot(self, mv_time, direction = 1):
        cmd_vel = Twist()
        cmd_vel.linear.x = direction * 0.2 

        sleep_time = 0.1 # time interval at which cmd_vel is published

        for i in range(int(mv_time / sleep_time)):
            self.publisher_.publish(cmd_vel)
            time.sleep(sleep_time)
    

    # rotate the robot for mv_time
    # rotate the robot clockwise: direction = 1, counter-clockwise: direction = -1
    def rotate_robot(self, mv_time, direction = 1):
        cmd_vel = Twist()
        cmd_vel.angular.z = direction * 0.2 

        sleep_time = 0.1 # time interval at which cmd_vel is published

        for i in range(int(mv_time / sleep_time)):
            self.publisher_.publish(cmd_vel)
            time.sleep(sleep_time)
    
class ElevatorHandler(Node):
    def __init__(self):
        super().__init__('elevator_handler_node')
        self.publisher_up_ = self.create_publisher(String, 'elevator_up',1)
        self.publisher_down_ = self.create_publisher(String, 'elevator_down', 1)
    
    # lift the elevator so the shelf is attached to the robot
    def elevator_up(self):
        self.publisher_up_.publish(String())
    
    # release the shelf
    def elevator_down(self):
        self.publisher_down_.publish(String())

class FootprintUpdater(Node):
    def __init__(self):
        super().__init__('footprint_updater_node')
        self.publisher_global_costmap_ = self.create_publisher(Polygon, 'global_costmap/footprint', 1)
        self.publisher_local_costmap_ = self.create_publisher(Polygon, 'local_costmap/footprint', 1)
        self.polygon_ = Polygon()
        self.client_ = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request_ = SetParameters.Request()

    # publish to the global and local costmaps one of the robot footprint depending on the shape parameter
    def publish_footprint(self, shape='robot'):
        r_s_inf_radius = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.4)
        r_inf_radius = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)
        if(shape == 'robot'):
            self.polygon_.points = robot_fp
            self.request_.parameters = [Parameter(name='inflation_layer.inflation_radius', value=r_inf_radius)]
        elif(shape == 'robot+shelf'):
            self.polygon_.points = robot_shelf_fp
            self.request_.parameters = [Parameter(name='inflation_layer.inflation_radius', value=r_s_inf_radius)]
        else:
            print('unrecognized shape. Possible shapes are :\'robot\'(for the basic robot footprint)'
            'or \'robot+shelf\'(for the robot + the shelf)')
            return -1
        
        self.publisher_global_costmap_.publish(self.polygon_)
        self.publisher_local_costmap_.publish(self.polygon_)
        self.future = self.client_.call_async(self.request_)
        return 0

def nav_to_pose(pose, navigator):
    destination_pose = PoseStamped()
    destination_pose.header.frame_id = 'map'
    destination_pose.header.stamp = navigator.get_clock().now().to_msg()
    destination_pose.pose.position.x = pose.position[0]
    destination_pose.pose.position.y = pose.position[1]
    destination_pose.pose.orientation.x = 0.0
    destination_pose.pose.orientation.y = 0.0
    destination_pose.pose.orientation.z = pose.orientation[2]
    destination_pose.pose.orientation.w = pose.orientation[3]
    print('Received request to move to ' + pose.name + '.')
    navigator.goToPose(destination_pose)

    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + pose.name + ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
    
    return navigator.getResult()

def main(argv):
    rclpy.init()

    # instantiates the navigator and all the node required for the task 
    navigator = BasicNavigator()
    mv_robot = ApproachShelf()
    elv_handler = ElevatorHandler()
    fp_updater = FootprintUpdater()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = initial_position.position[0]
    initial_pose.pose.position.y = initial_position.position[1]
    initial_pose.pose.orientation.z = initial_position.orientation[2]
    initial_pose.pose.orientation.w = initial_position.orientation[3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to be fully activated 
    navigator.waitUntilNav2Active()

    result = nav_to_pose(shelf_position, navigator)

    if result == TaskResult.CANCELED:
        print('Task was canceled. Returning to staging point...')
        result = nav_to_pose(initial_position, navigator)
        exit(1)

    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)
    
    elif result == TaskResult.SUCCEEDED:
        print('The robot arrived to its destination!')
        time.sleep(1)

    mv_robot.call_approach_shelf_server()
    while rclpy.ok():
        rclpy.spin_once(mv_robot)
        if mv_robot.future.done():
            try:
                response = mv_robot.future.result()
            except Exception as e:
                mv_robot.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                mv_robot.get_logger().info(f'Result of service call: {response.complete}')                
            break
    if response.complete != True:
        exit(-1)

    mv_robot.move_robot(2.5) # move the robot under the shelf
    time.sleep(1.5)
    elv_handler.elevator_up() # lift the elevator to attach the shelf to the robot
    fp_updater.publish_footprint('robot+shelf') # set the new footprint of the robot

    time.sleep(1)

    # # move the robot to avoid it getting stuck
    # mv_robot.move_robot(6, -1)  
    # mv_robot.rotate_robot(13, -1)

    # result = nav_to_pose(intermediate_position, navigator)

    # if result == TaskResult.CANCELED:
    #     print('Task was canceled. Returning to staging point...')
    #     result = nav_to_pose(initial_position, navigator)
    #     exit(1)

    # elif result == TaskResult.FAILED:
    #     print('Task failed!')
    #     exit(-1)
    
    # elif result == TaskResult.SUCCEEDED:
    #     print('The robot arrived to its destination!')
    #     time.sleep(1)

    # result = nav_to_pose(shipping_position, navigator)

    # if result == TaskResult.CANCELED:
    #     print('Task was canceled. Returning to staging point...')
    #     result = nav_to_pose(initial_position, navigator)
    #     exit(1)

    # elif result == TaskResult.FAILED:
    #     print('Task failed!')
    #     exit(-1)
    
    # elif result == TaskResult.SUCCEEDED:
    #     print('The robot arrived to its destination!')
    #     time.sleep(1)
    
    # elv_handler.elevator_down() # release the shelf
    # fp_updater.publish_footprint('robot')
    # time.sleep(0.5)

    # # move the robot backward to unstuck it
    # mv_robot.move_robot(7, -1)

    # result = nav_to_pose(initial_position, navigator)

    # if result == TaskResult.CANCELED:
    #     print('Task was canceled. Returning to staging point...')
    #     result = nav_to_pose(initial_position, navigator)
    #     exit(1)

    # elif result == TaskResult.FAILED:
    #     print('Task failed!')
    #     exit(-1)
    
    # elif result == TaskResult.SUCCEEDED:
    #     print('The robot arrived to its destination!')
    #     time.sleep(1)
    
    print('mission done!')

    exit(0)

if __name__ == '__main__':
    main(sys.argv)