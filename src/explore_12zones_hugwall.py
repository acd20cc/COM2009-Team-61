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
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Explorer:
    TASK_TIME_SEC = 90.0
    DEFAULT_VEL = 0.26
    DEFAULT_ANGULAR_VEL = 0.8
    DEFAULT_ANGULAR_VELV2 = 0.1
    ARENA_HORIZONTAL = 3.4
    ARENA_VERTICAL = 3.4
    REQ_DIST = 0.36

    def __init__(self):
        self.node_name = 'explore_12zones_node'
        rospy.init_node(self.node_name, anonymous = True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.rate = rospy.Rate(10)  # 10Hz
        self.ctrl_c = False
        #coordiantes roughly translate to top-left, top-right, bot-right, got,left
        self.arena_edges = [[-1.5,-1.5], [-1.5,1.5], [1.5, 1.5], [1.5,-1.5]]
        self.vel = Twist()
        self.facing = ""
        self.edge = False
        self.side = False
        self.wall = False
        self.atSide = False
        self.distant_obstacle = False
        self.obstacle_detected = False
        self.rotation_direction = 1
        self.min_distance = 0
        self.left_edge_closest = 0
        self.right_edge_closest = 0
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
        if(not self.atSide):
            self.find_side()

    #receive odometry data and put in a better format
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

    #removes zero values in an array
    def no_zero_min(self, arc):
        for i in range(len(arc)):
            if (arc[i] < 0.0001):
                arc[i] = 5
        return arc


    # front arc refers to the 'x' coordinate
    def process_front_arc(self, ranges):
        # process data from LiDAR data into an array
        #             (taken from leturer's notes)
        left_arc = ranges[0:31]
        right_arc = ranges[-30:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        # calculating positions/angles of nearby obstacles 
        #            (in correspondance to the robot)
        front_arc = self.no_zero_min(front_arc) #remove 0 values in scan data
        self.min_distance = front_arc.min() # min distance to an obstacle

        arc_angles = np.arange(-30, 31)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]
        self.left_edge_closest = np.array(front_arc[0:5]).min()
        self.right_edge_closest = np.array(front_arc[-5:]).min()

        self.left_wider_edge_closest = np.array(front_arc[0:10]).min()
        self.right_wider_edge_closest = np.array(front_arc[-10:]).min()

    def find_side(self):
        self.vel.linear.x = self.DEFAULT_VEL

        # if distant obstacle detected, swerve slightly to avoid it
        if self.min_distance < 1:
            if not self.distant_obstacle:
                self.rotation_direction = 1
                self.distant_obstacle = True
                rospy.loginfo(f"Obstacle detected at distance of {self.min_distance:.3f} ")
        else:
            self.distant_obstacle = False

        # if obstacle is within certain distance, change robot direction
        if self.min_distance < 0.5:
            if not self.obstacle_detected:
                self.rotation_direction = 1
                self.obstacle_detected = True
                rospy.loginfo(f"Obstacle detected at distance of {self.min_distance:.3f} ")
        else:
            self.obstacle_detected = False

        # if robot aprroaching the edge of the arena, randomly change robot's direction
        if self.left_edge_closest < 0.32 and self.right_edge_closest < 0.32:
            if self.left_edge_closest < self.right_edge_closest:
                self.rotation_direction = -1
            else:
                self.rotation_direction = 1
            self.edge = True
            self.obstacle_detected = True
        else:
            self.edge = False

    #returns true or false if an obstacle is the side of arena
    def at_side(self, left_side, right_side, req_dist):
        if ((np.absolute(self.posx) > 1.70 - req_dist) | (np.absolute(self.posx) > 1.70 - req_dist)):
            self.atSide = True
            self.facing = self.is_lr_side(self.left_side_arc, self.right_side_arc)
        else: 
            self.atSide = False
        left_average = 0
        right_average = 0
        for i in left_side:
            left_average += i
        for i in right_side:
            right_average += i
        left_average /= len(left_side)
        right_average /= len(right_side)
        if ((left_average < 1.1*req_dist)):
            self.atSide = True
            self.facing = self.is_lr_side(self.left_side_arc, self.right_side_arc)
        else:
            self.atSide = False
        if ((right_average < 1.1*req_dist)):
            self.atSide = True
            self.facing = self.is_lr_side(self.left_side_arc, self.right_side_arc)
        else:
            self.atSide = False

    def is_lr_side(self, left_side, right_side):
        left_average = 0
        right_average = 0
        for i in left_side:
            left_average += i
        for i in right_side:
            right_average += i
        left_average /= len(left_side)
        right_average /= len(right_side)
        if ((left_average < right_average)):
            return "Left"
        return "Right"

    #method for mostly when robot reaches end of plank obstacle and no more wall next to it
    #sets self.wall false if expected wall not next to it
    def wall_gone(self, req_dist):
        if (self.facing == "Left"):
            self.wall = True
            if(self.front_half_left > 1.5 * req_dist):
                self.wall = False
        elif (self.facing == "Right"):
            self.wall = True
            if(self.front_half_right > 1.5 * req_dist):
                self.wall = False

    def handle_corner(self):
        print("handle corner called")
        if self.left_edge_closest < 0.32 and self.right_edge_closest < 0.32:
            if(self.facing == "Left"):
                rospy.loginfo('Avoiding corner of arena')
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -1
                self.vel.linear.x = self.DEFAULT_VEL * 0
            else:
                rospy.loginfo('Avoiding corner of arena')
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * 1
                self.vel.linear.x = self.DEFAULT_VEL * 0
            self.cmd_vel_pub.publish(self.vel)
            rospy.sleep(self.DEFAULT_ANGULAR_VEL)

    #checks in obstacle(wall in this case) is infront and reacts appropriately
    def handle_wall(self):
        if self.min_distance < 0.37:
            self.vel.linear.x = 0.0
            self.rotation_direction = 1
        elif self.min_distance < 0.6:
            self.vel.linear.x = 0.16
            self.rotation_direction = 0.2
            print("handle wall called")
        #rotate other way if moving with left side against wall
        if(self.facing == "Left"):
            self.rotation_direction *= -1
        self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * self.rotation_direction

    #follows the side of arena
    def follow_side(self, req_dist):
        #checks if wall is gone, means either too far from wall or it has gone around plank
        #obstacle. effectively does a u turn if it has gone around a plank
        self.wall_gone(req_dist)
        self.vel.linear.x = self.DEFAULT_VEL
        side_arc = np.array(self.left_side_arc[::-1] + self.right_side_arc[::-1])
        self.min_side_distance = side_arc.min()

        #keeps required distance from wall
        if(self.facing == "Left"):
            if (self.min_side_distance < req_dist):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VELV2 * -0.3
            if (self.min_side_distance > req_dist):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VELV2 * 0.3
        else:
            if (self.min_side_distance < req_dist):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VELV2 * 0.3
            if (self.min_side_distance > req_dist):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VELV2 * -0.3
        
        #if accidently accelerating away from wall it slows down
        if (self.min_side_distance > 1.3*req_dist):
            self.vel.linear.x = self.DEFAULT_VEL * 0.7

        self.handle_wall()
        #if no wall it does extreme turn
        if(self.wall == False):
            self.vel.linear.x = 0.14
            if(self.facing == "Left"):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * 0.8
            else:
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -0.8
        #useless right now
        if (self.edge):
            self.handle_corner()

    # the side arc refers to the 'y' coordinate
    def process_side_arc(self, ranges):
        # process data from LiDAR data into an array
        #             (taken from leturer's notes)
        self.left_side_arc = self.no_zero_min(ranges[70:121])
        self.right_side_arc = self.no_zero_min(ranges[250:300])
        side_arc = np.array(self.left_side_arc[::-1] + self.right_side_arc[::-1])

        # calculating positions/angles of nearby obstacles 
        #            (in correspondance to the robot)

        self.min_side_distance = side_arc.min() # min distance to an obstacle

        #take front-facing side data
        self.front_half_left = np.array(self.left_side_arc[0:14]).min()
        self.front_half_right = np.array(self.right_side_arc[-14:]).min()

        side_arc_angles = np.arange(-50, 51)
        self.min_side_position = side_arc_angles[np.argmin(side_arc)]

        if(not self.atSide):
            self.at_side(self.left_side_arc, self.right_side_arc, self.REQ_DIST)
        # if robot detected within a certain threshold distance
        if self.min_side_distance < 0.25:
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
        #adjust velocity and angular vel if distant object detected
        if self.distant_obstacle:
            self.vel.linear.x *= 0.8
            if(self.left_wider_edge_closest > self.right_wider_edge_closest):
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * 0.35
            else:
                self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -0.35
        else:
            self.vel.linear.x = self.DEFAULT_VEL

        # adjusting linear velocity depending on obstacle distance
        if self.min_distance < 0.35 or self.min_side_distance < 0.2:
            self.vel.linear.x = 0.0
        elif self.min_distance < 0.6:
            self.vel.linear.x *= 0.7

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

            if self.min_side_distance < 0.4:
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
        #self.vel.angular.z = random.uniform(-1, 1) * self.rotation_direction


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
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        if (not self.obstacle_detected) & (not self.atSide):
            #self.find_side()
            self.levy_flight_step()
        if self.atSide:
            self.follow_side(self.REQ_DIST)
        elif self.obstacle_detected:
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
        rate = rospy.Rate(10) 
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
