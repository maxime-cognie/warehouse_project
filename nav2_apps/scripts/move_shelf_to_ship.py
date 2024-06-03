#!/usr/bin/env python3

import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from std_msgs.msg import String
from geometry_msgs.msg import Polygon
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy
from typing import NamedTuple

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class Goal(NamedTuple):
    name: str
    position: list
    orientation: list

# Initial position of the robot
initial_position = Goal(
    'initial_position',
    [-1.2, 2.3, 0.0],
    [0.0, 0.0, 0.0, 1.0])

# Shelf positions for picking
shelf_position = Goal(
    'shelf_position',
    [4.8, 1.1, 0.0],
    [0.0, 0.0, 0.7660444, 0.6427876])

# Shipping destination for picked products
shipping_position = Goal(
    'shipping_position',
    [1.5, 3.55, 0.0],
    [0.0, 0.0, -0.785375, 0.785375])

robots_route = [
    shelf_position,
    # shipping_position,
    # initial_position
    ]

robot_fp = []

robot_shelf_fp = [
    [0.425, 0.4, 0.0], 
    [0.425, -0.4, 0.0], 
    [-0.425, -0.4, 0.0], 
    [-0.425, 0.4, 0.0]
]

class ApproachShelf(Node):
    def __init__(self):
        super().__init__('approach_shelf_node')
        self.publisher_ = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 1)
       

    # move the robot backward until it is under the shelf
    def move_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2 

        mv_time = 5 # duration of the robot movement (sec)
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
    
    # publish to the global and local costmaps one of the robot footprint depending on the shape parameter
    def publish_footprint(self, shape='robot'):
        polygon = Polygon()
        if(shape == 'robot'):
            polygon.points.append(robot_fp)
        elif(shape == 'robot+shelf'):
            polygon.points.append(robot_shelf_fp)
        else:
            print('unrecognized shape. Possible shapes are :\'robot\'(for the basic robot footprint)'
            'or \'robot+shelf\'(for the robot + the shelf)')
            return -1
        
        self.publisher_global_costmap_.publish(polygon)
        self.publisher_local_costmap_.publish(polygon)
        return 0



def nav_to_pose(pose, navigator):
    destination_pose = PoseStamped()
    destination_pose.header.frame_id = 'map'
    destination_pose.header.stamp = navigator.get_clock().now().to_msg()
    destination_pose.pose.position.x = pose.position[0]
    destination_pose.pose.position.y = pose.position[1]
    destination_pose.pose.orientation.z = pose.orientation[2]
    destination_pose.pose.orientation.w = pose.orientation[3]
    print('Received request to move to' +pose.name + '.')
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


def main():
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
    initial_pose.pose.orientation.w = initial_position.orientation[3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to be fully activated 
    navigator.waitUntilNav2Active()

    for pose in robots_route:
        result = nav_to_pose(pose, navigator)

        if result == TaskResult.CANCELED:
            print('Task was canceled. Returning to staging point...')
            result = nav_to_pose(initial_position, navigator)
            exit(1)

        elif result == TaskResult.FAILED:
            print('Task failed!')
            exit(-1)
        
        elif result == TaskResult.SUCCEEDED:
            print('The robot arrived to its destination!')
            time.sleep(3)

    mv_robot.move_robot()

    elv_handler.elevator_up()

    fp_updater.publish_footprint('robot+shelf')
    
    print('mission done!')

    exit(0)


if __name__ == '__main__':
    main()