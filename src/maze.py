#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class maze:

    def __init__(self):
        self.node_name = "maze_node"

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # default distance
        self.min_left = 0
        self.min_front= 0
        self.min_right = 0

        # default velocity into max
        self.max_velocity = 0.26

        # default front distance
        self.front_dist = 0 
        self.left_dist = 0
        self.right_dist = 0
        
        self.vel = Twist()

        #default for stopping the code
        self.ctrl_c = False



        rospy.on_shutdown(self.shutdown_hook)
    
    # default function to kill the program when contrl c is entered
    def shutdown_hook(self):
        self.cmd_vel_pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self, scan_data):

        # Extract the the last 90 measurements demonstrate right side arc
        right_arc = scan_data.ranges[-90:]
        # Extract the first 91 measurements demonstrate the left side arc.
        left_arc = scan_data.ranges[0:91]
        # Merge and reverse left and right side arc measurements to form front arc.
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        #Set array right side between 120 to 150
        self.right_dist = np.array(front_arc[120:150]).min()
        #Set array front side between 70 to 110
        self.front_dist= np.array(front_arc[70:110]).min()
        #Set array left side between 30 to 60
        self.left_dist = np.array(front_arc[30:60]).min()

        
    def main(self):
        """
        Main loop for the maze-solving algorithm.
        """
        while not self.ctrl_c:
            min_dist = self.front_dist > 0.39
            
            # If the distance of right side is less than 0.3 and distance front side is bigger than 0.39 turn right
            # right_distance is used to prioritise the right side from two ways.
            if self.right_dist < 0.3 and min_dist:
                print("Chose to turn Left")
                self.vel.linear.x = self.max_velocity 
                self.vel.angular.z = 1

            # If the distance on the right is between 0.3 and 0.4 units and min_dist is true, proceed forward.
            elif 0.3 < self.right_dist < 0.4 and min_dist:
                print("Chose to go front")
                self.vel.linear.x = self.max_velocity
                self.vel.angular.z = 0

            # If right side is open, turn to the right side
            elif self.right_dist > 0.3 and min_dist:
                print("Chose to turn right")
                self.vel.linear.x = self.max_velocity
                self.vel.angular.z = -1

            # else turn 180 degree until free space is scanned.
            else:
                print("Obstacle detected in front, turning the object")
                self.vel.linear.x = 0
                self.vel.angular.z = 1
                self.cmd_vel_pub.publish(self.vel)

            # publish the movement
            self.cmd_vel_pub.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    node = maze()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
