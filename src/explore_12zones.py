#!/usr/bin/env python3

"""
Team 61
------------------------
Task 2: Explore 4 x 4m arena, avoid obsticles
        and enter as main 1 x 1m zones as possible
        (there are 12 in total)
------------------------


Things I want to add:
    - a move advanced navigation algorithm (currently it does it randomly)
        see move() for this
    - a checker to see how many zones it has entered? (the arena size stays the
        same so shouldn't be two hard? Can hard program the values?)

"""

import rospy
import random
import numpy as np

from time import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Explorer:
    TASK_TIME_SEC = 90.0
    DEFAULT_VEL = 0.26
    DEFAULT_ANGULAR_VEL = 0.7

    def __init__(self):
        self.node_name = 'explore_12zones_node'
        rospy.init_node(self.node_name, anonymous = True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.rate = rospy.Rate(30)  # 30Hz
        self.ctrl_c = False
        
        self.vel = Twist()
        self.edge = False
        self.side = False
        self.obstacle_detected = False
        self.rotation_direction = 1
        self.timer = 0.0 # for timing the tasks length

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    """
    ------------------------
    Process data from LiDAR scanners/ detect nearby obstacles
    ------------------------
    """

    # receive data from robot
    def laser_callback(self, scan_data):
        self.process_front_arc(scan_data.ranges)
        self.process_side_arc(scan_data.ranges)


    # front arc refers to the 'x' coordinate
    def process_front_arc(self, ranges):
        # process data from LiDAR data into an array
        #             (taken from leturer's notes)
        left_arc = ranges[0:31]
        right_arc = ranges[-30:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        # calculating positions/angles of nearby obstacles 
        #            (in correspondance to the robot)
        self.min_distance = front_arc.min() # min distance to an obstacle
        arc_angles = np.arange(-30, 31)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]
        left_edge_closest = np.array(front_arc[0:5]).min()
        right_edge_closest = np.array(front_arc[-5:]).min()

        # if obstacle is within certain distance, randomly change robot's direction
        if self.min_distance < 0.6:
            if not self.obstacle_detected:
                self.rotation_direction = random.choice([1, -1])
                self.obstacle_detected = True
                rospy.loginfo(f"Obstacle detected at distance of {self.min_distance:.3f} ")
        else:
            self.obstacle_detected = False

        # if robot aprroaching the edge of the arena, randomly change robot's direction
        if left_edge_closest < 0.8 and right_edge_closest < 0.8:
            if left_edge_closest < right_edge_closest:
                self.rotation_direction = -1
            else:
                self.rotation_direction = 1
            self.edge = True
            self.obstacle_detected = True
        else:
            self.edge = False


    # the side arc refers to the 'y' coordinate
    def process_side_arc(self, ranges):
        # process data from LiDAR data into an array
        #             (taken from leturer's notes)
        left_side_arc = ranges[70:121]
        right_side_arc = ranges[250:300]
        side_arc = np.array(left_side_arc[::-1] + right_side_arc[::-1])

        # calculating positions/angles of nearby obstacles 
        #            (in correspondance to the robot)
        self.min_side_distance = side_arc.min() # min distance to an obstacle
        side_arc_angles = np.arange(-50, 51)
        self.min_side_position = side_arc_angles[np.argmin(side_arc)]

        # if robot detected within a certain threshold distance
        if self.min_side_distance < 0.35:
            if not self.obstacle_detected:
                rospy.loginfo(f"Obstacle edge detected at distance of {self.min_side_distance:.3f}")
                self.obstacle_detected = True
            self.side = True
        else:
            self.side = False

    """
    ------------------------
    Use processed data to then avoid the obstacles 
    ------------------------
    """

    def handle_obstacle(self):
        # adjusting linear velocity depending on obstacle distance
        if self.min_distance < 0.40 or self.min_side_distance < 0.2:
            self.vel.linear.x = 0.0
        elif self.min_distance < 0.5:
            self.vel.linear.x = 0.15
        else:
            self.vel.linear.x = 0.20

        # when the robot detect's that it is near the edge of the arena
        #            (for this we use angular velocity instead)
        if self.edge:
            rospy.loginfo('Avoiding edge of arena')
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * self.rotation_direction
            self.cmd_vel_pub.publish(self.vel)
            rospy.sleep(self.DEFAULT_ANGULAR_VEL)
        elif self.closest_object_position > 17:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * 1
        elif self.closest_object_position < -17:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -1
        else:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * self.rotation_direction

        # ajust angular velocity if obstacles are detected on the side
        if self.side:
            self.rotation_direction = -1 if self.min_side_position < 0 else 1

            if self.min_side_distance < 0.3:
                self.vel.angular.z = 0.2 * self.rotation_direction
            elif self.min_side_distance < 0.4:
                self.vel.angular.z = 0.2 * self.rotation_direction
            else:
                self.vel.angular.z = 0.5 * self.rotation_direction
        
    """
    ------------------------
    implementing the levey flight search strategy 
    ------------------------
    """

    def levy_flight_step(self):
        # Probability of taking a long jump
        long_jump_prob = 0.1  # 10% chance of a long jump
        max_long_jump = 0.26  # Maximum velocity for long jumps
        normal_step = 0.2  # Normal step velocity


        # Randomly decide if this move will be a long jump or a normal step
        if random.random() < long_jump_prob:
            # Long jump
            step_length = max_long_jump
        else:
            # Normal step
            step_length = normal_step

        # Apply the step length to the linear velocity
        self.vel.linear.x = step_length

        # Introduce randomness in angular velocity for direction change
        self.vel.angular.z = random.uniform(-1, 1) * self.rotation_direction



    """
    ------------------------
    the robot's general movements
    ------------------------
    """

    # function to stop the robot when program terminates
    def shutdown(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)
        self.ctrl_c = True
        rospy.loginfo('** Terminating program **')

    # function to move the robot
    def move(self):
        # this is where to put the algorithm to determine the robot's navigation path
        #            (currently it just randomly navigates)
        '''if not self.obstacle_detected:
            self.vel.linear.x = self.DEFAULT_VEL
            # self.vel.angular.z = 0.2 * self.rotation_direction
        # an obstacle has been encountered, now go to the handle_obstacle() function
        else:
            self.handle_obstacle()'''
        if not self.obstacle_detected:
            self.levy_flight_step()
        else:
            self.handle_obstacle()
        


    """
    ------------------------
    the 'main' function:
        - initiates the robot's movements/ calls move()
        - records task completion time (if exceeded terminate)
        - checks if ctrl c has been pressed
    ------------------------
    """

    def explore(self):
        start = time() # record time it takes for robot to complete the task

        while not self.ctrl_c and self.timer <= self.TASK_TIME_SEC:
            self.move()
            self.cmd_vel_pub.publish(self.vel) # publish velocity commands
            self.rate.sleep()

            approx_time_sec = round(self.timer, 1)
            if approx_time_sec % 10 == 0: # print the time every 10 seconds
                rospy.loginfo(f'Stop watch time (seconds): {approx_time_sec:.1f}')

            self.timer = time() - start

        rospy.loginfo('Task 2 completed')
        self.shutdown()


if __name__ == '__main__':
    node = Explorer()

    try: # if the program fails to load
        node.explore()
    except rospy.ROSInterruptException:
        pass