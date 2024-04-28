
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


class Task4():
    #roslaunch com2009_simulations empty_arena.launch 
    TASK_TIME_SEC = 180
    # Constants
    ROBOT_WIDTH = 30
    OBSTCALE_FREE = 40


    def __init__(self):
        node_name = "task4"
        self.node_name = rospy.get_name()
        rospy.init_node(node_name, anonymous=True)

        # Topics subscribed to
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        #Initalize variables 
        self.rate = rospy.Rate(5)
        self.ctrl_c = False
        self.vel = Twist()
        self.max_range = 10.0
        self.prev_ranges = None
        self.timer = 0.0 

        self.min_left = 0
        self.min_front= 0
        self.min_right = 0

        self.front_dist = 0 
        self.left_dist = 0
        self.right_dist = 0

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    # Function to stop the robot when program terminates
    def shutdown(self):
        # Stop the robot
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)
        self.ctrl_c = True
        rospy.loginfo('** Terminating program **')
        self.ctrl_c = True

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

    def laser_callback(self, scan_data):
        # Callback function to handle incoming laser scan data
        curr_ranges = np.array(scan_data.ranges)

        self.right_dist = np.array(curr_ranges[120:150]).min()
        self.front_dist= np.array(curr_ranges[70:110]).min()
        self.left_dist = np.array(curr_ranges[30:60]).min()
        
        # Check if previous ranges are available every 5 seconds
        approx_time_sec = round(self.timer, 1) 
        if approx_time_sec % 3 == 0: 
            if self.prev_ranges is not None:
                # Detect door passing
                door_detected, door_side = self.detect_door_passing(self.prev_ranges, curr_ranges)
                if door_detected:
                    print(f"Door passing detected on the {door_side} side!")
                    self.enter_doorway(door_side)
                else: 
                    print(f"No Door detected!")
        
        # Update previous ranges for the next iteration
        self.prev_ranges = curr_ranges.copy()

    def detect_door_passing(self, prev_ranges, curr_ranges):
        
        # Calculate differences in ranges
        diff_left = np.abs(curr_ranges[60:90] - prev_ranges[60:90])
        diff_right = np.abs(curr_ranges[270:300] - prev_ranges[270:300])

        threshold = 0.5 
        
        # Check for sudden changes indicating door passing
        if np.any(diff_left > threshold) and np.any(diff_right > threshold):
            door_side = "both"
            return True, door_side 
        elif np.any(diff_left > threshold) or np.any(diff_right > threshold):
            # Identify the side of the door (left or right)
            if np.any(diff_left > threshold):
                door_side = "left"
            else:
                door_side = "right"
    
            return True, door_side  # Door passing detected
        else:
            return False, None  # No door passing detected

    def enter_doorway(self, door_side):
        # Stop the robot
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)

        rospy.loginfo(f"Entering through the {door_side} doorway...")

        # Enter the doorway based on the side
        if door_side == "left" or door_side == "both":
            self.vel.angular.z = 0.3
            self.vel.linear.x = 1.2

        else:
            self.vel.angular.z = -0.3
            self.vel.linear.x = 0.1

        # Publish the velocity commands to make the robot enter the doorway
        self.cmd_vel_pub.publish(self.vel)
        rospy.sleep(6)  

        # Stop the robot after turning
        rospy.loginfo(f"Done turning into doorway")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel)
        

    def move(self):
        self.vel.linear.x = 0.1


    def main(self):
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

        rospy.loginfo('Task 4 completed')
        self.shutdown()

if __name__ == '__main__':
    node = Task4()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
