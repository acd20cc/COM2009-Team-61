#!/usr/bin/env python3

import rospy
import random
import numpy as np
import waffle 


from time import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees

node_name = "navigation"

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(10) # hz
rospy.loginfo(f"{node_name}: Initialised.")

lidar = waffle.Lidar(debug = True)

class Nav:
    TASK_TIME_SEC = 180.0
    DEFAULT_VEL = 0.26
    DEFAULT_ANGULAR_VEL = 0.9


    def __init__(self):
        self.node_name = 'navigation'
        rospy.init_node(self.node_name, anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.rate = rospy.Rate(10)  # 10Hz
        self.ctrl_c = False

        self.vel = Twist()
        self.edge = False
        self.side = False
        self.obstacle_detected = False
        self.rotation_direction = 1
        self.timer = 0.0  # for timing the tasks length

        self.positions = []  # Initialize an empty array to store positions
        self.last_check_time = time()  # Initialize last check time
        self.last_posx = 0.0
        self.last_posy = 0.0
        self.last_yaw = 0.0
        self.position_change = True

        self.min_side_distance = 0

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"The '{self.node_name}' node is active...")
    
    """
    ------------------------
    Process data from Odom for postion of robot 
    ------------------------
    """

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


    """
    ------------------------
    Process data from LiDAR scanners/ detect nearby obstacles
    ------------------------
    """

    # receive data from robot
    def laser_callback(self, scan_data):
        self.process_front_arc(scan_data.ranges)
        self.process_side_arc(scan_data.ranges)

    def no_zero_min(self, arc):
        for i in range(len(arc)):
            if arc[i] < 0.0001:
                arc[i] = 2
        return arc

    # front arc refers to the 'x' coordinate
    def process_front_arc(self, ranges):
        left_arc = ranges[0:31]
        right_arc = ranges[-30:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        front_arc = self.no_zero_min(front_arc)

        self.min_distance = front_arc.min()  # min distance to an obstacle

        arc_angles = np.arange(-30, 31)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]
        left_edge_closest = np.array(front_arc[0:5]).min()
        right_edge_closest = np.array(front_arc[-5:]).min()

        # if obstacle is within certain distance, randomly change robot's direction
        if self.min_distance < 0.6:
            if not self.obstacle_detected:
                self.rotation_direction = 1
                self.obstacle_detected = True
                rospy.loginfo(f"Obstacle detected at distance of {self.min_distance:.3f} ")

        else:
            self.obstacle_detected = False

        # if robot approaching the edge of the arena, randomly change robot's direction
        if left_edge_closest < 0.3 and right_edge_closest < 0.3:
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
        left_side_arc = ranges[70:121]
        right_side_arc = ranges[250:300]
        side_arc = np.array(left_side_arc[::-1] + right_side_arc[::-1])

        side_arc = self.no_zero_min(side_arc)

        self.min_side_distance = side_arc.min()  # min distance to an obstacle
        side_arc_angles = np.arange(-50, 51)
        self.min_side_position = side_arc_angles[np.argmin(side_arc)]

        # if robot detected within a certain threshold distance
        if self.min_side_distance < 0.5:
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
        if self.min_distance < 0.40 or self.min_side_distance < 0.2:
            self.vel.linear.x = 0.0
        elif self.min_distance < 0.5:
            self.vel.linear.x = 0.15
        else:
            self.vel.linear.x = 0.20

        if self.edge:
            rospy.loginfo('Avoiding edge of arena')
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * self.rotation_direction
            self.cmd_vel_pub.publish(self.vel)
            rospy.sleep(self.DEFAULT_ANGULAR_VEL)
        elif self.closest_object_position > 17:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -1
        elif self.closest_object_position < -17:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * -1
        else:
            self.vel.angular.z = self.DEFAULT_ANGULAR_VEL * self.rotation_direction

        if self.side:
            self.rotation_direction = -1 if self.min_side_position < 0 else 1

            if self.min_side_distance < 0.5:
                self.vel.angular.z = 0.2 * self.rotation_direction
            elif self.min_side_distance < 0.6:
                self.vel.angular.z = 0.2 * self.rotation_direction
            else:
                self.vel.angular.z = 0.5 * self.rotation_direction

    """
    ------------------------
    Implement the robot's general movements
    ------------------------
    """

    def move(self):
        if not self.obstacle_detected:
            if not self.position_change:                
                lidar.subsets.show()
                
            else:
                self.random_step()  # Perform random step if position changed recently
        else:
            if not self.position_change:
                lidar.subsets.show()
            else:
                self.handle_obstacle()  # Handle obstacle if detected

    """
    ------------------------
    Implementing the random search strategy
    ------------------------
    """

    def random_step(self):
        long_jump_prob = 0.1  # 10% chance of a long jump
        max_linear = 0.26  # Maximum velocity for long jumps
        angle = 0 


        if random.random() < long_jump_prob:
            angle = random.uniform(-1.82, 1.82) * self.rotation_direction
        else:
            angle = random.uniform(-1, 1) * self.rotation_direction

        self.vel.linear.x = max_linear
        self.vel.angular.z = angle
    """
    ------------------------
    using the waffle code given 
    ------------------------
    """

    """
    ------------------------
    get the postion differnence every 10 seconds  
    ------------------------
    """
    def check_position_difference(self):
        # Calculate absolute difference in position and yaw
        posx_diff = abs(self.posx - self.last_posx)
        posy_diff = abs(self.posy - self.last_posy)
        yaw_diff = abs(self.yaw - self.last_yaw)

        # Log the differences
        rospy.loginfo(f'Position difference: ({posx_diff}, {posy_diff}), Yaw difference: {yaw_diff}')

        # Store the current position in the array
        self.positions.append((self.posx, self.posy))

        # Update last recorded values
        self.last_posx = self.posx
        self.last_posy = self.posy
        self.last_yaw = self.yaw
        self.last_check_time = time()

    def check_position_change(self):
        posx_diff = abs(self.posx - self.last_posx)
        posy_diff = abs(self.posy - self.last_posy)
        angle_change_array = [0, 30, 50, 78, 103, 30, 50, 78, 103] 
        side = "none"

        # Check if absolute position difference is less than 0.5 for both x and y
        if posx_diff < 0.5 and posy_diff < 0.5:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            rospy.loginfo("Difference in position is less than 0.5. Robot barely changed its position.")
            self.position_change = False 

            # Call get_all_distances to get all distances and their corresponding subparts
            distances, subparts = lidar.get_all_distances_with_subparts()

            # Initialize variables to hold the greatest distance and its corresponding subpart
            greatest_distance = float('-inf')
            greatest_subpart = ""
            angle_change = 0 

            # Loop through the distances and subparts arrays to find the greatest distance and its corresponding subpart
            for i, distance in enumerate(distances):
                if distance > greatest_distance:
                    greatest_distance = distance
                    greatest_subpart = subparts[i]
                    angle_change = angle_change_array[i]
                    if i > 3:
                        side = "right"


            # Check if the greatest distance is less than 0.5
            if greatest_distance < 0.5:
                rospy.loginfo("Greatest distance very close to obstcale.")

            # Print the greatest distance and its corresponding subpart
            rospy.loginfo(f"Greatest distance: {greatest_distance}, Subpart: {greatest_subpart}")

            # Turn the robot using the calculated angle change
            if side == "right":
                self.turn_at_velocity(-self.DEFAULT_ANGULAR_VEL, angle_change)
                
            else:
                self.turn_at_velocity(self.DEFAULT_ANGULAR_VEL, angle_change)

    def turn_at_velocity(self, angular_velocity, angle_change):

        # Calculate the duration based on the angle change and angular velocity
        duration = abs(angle_change / angular_velocity)

        start_time = rospy.get_time()  # Record the start time
        end_time = start_time + duration  # Calculate the end time

        # Create a Twist message to publish velocity commands
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity  # Set the angular velocity

        # Publish velocity commands until the desired angle change is reached or an obstacle is detected
        while rospy.get_time() < end_time and not self.obstacle_detected:
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot after the desired angle change or obstacle detection
        twist_msg.angular.z = 0.0  # Set angular velocity to zero
        self.cmd_vel_pub.publish(twist_msg)  # Publish the stop command

        # Calculate the time taken for the turn
        time_taken = abs(angle_change / angular_velocity)

    
    """
    ------------------------
    Implement the 'main' function:
        - initiates the robot's movements/calls move()
        - records task completion time (if exceeded terminate)
        - checks if ctrl c has been pressed
    ------------------------
    """

    def explore(self):
        start = time()  # Record time it takes for robot to complete the task

        while not self.ctrl_c and self.timer <= self.TASK_TIME_SEC:
            self.move()
            self.cmd_vel_pub.publish(self.vel)  # Publish velocity commands
            self.rate.sleep()

            # Check odometry difference every 10 seconds
            if time() - self.last_check_time >= 10:
                self.check_position_difference()  # Call the function to check position difference
                self.check_position_change()

            # Update timer
            self.timer = time() - start

        rospy.loginfo('Task completed')
        self.shutdown()

    # Function to stop the robot when program terminates
    def shutdown(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)
        self.ctrl_c = True
        rospy.loginfo('** Terminating program **')

if __name__ == '__main__':
    node = Nav()

    try:
        node.explore()
    except rospy.ROSInterruptException:
        pass
