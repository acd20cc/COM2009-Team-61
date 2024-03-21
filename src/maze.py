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
        
        self.min_left = 0
        self.min_front= 0
        self.min_right = 0

        self.front_dist = 0 
        self.left_dist = 0
        self.right_dist = 0
        
        self.vel = Twist()
        self.ctrl_c = False
        

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.cmd_vel_pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self, scan_data):

        right_arc = scan_data.ranges[-90:]
        left_arc = scan_data.ranges[0:91]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.right_dist = np.array(front_arc[120:150]).min()
        self.front_dist= np.array(front_arc[70:110]).min()
        self.left_dist = np.array(front_arc[30:60]).min()

        
    def main(self):
        """
        Main loop for the maze-solving algorithm.
        """
        while not self.ctrl_c:
            min_dist = self.front_dist > 0.42
            
            if self.right_dist < 0.28 and min_dist:
                print("Chose to turn Left")
                self.vel.linear.x = 0.26 
                self.vel.angular.z = 1
            elif 0.28 < self.right_dist < 0.4 and min_dist:
                print("Chose to go front")
                self.vel.linear.x = 0.26
                self.vel.angular.z = 0
            elif self.right_dist > 0.3 and min_dist:
                print("Chose to turn right")
                self.vel.linear.x = 0.26
                self.vel.angular.z = -1
            else:
                print("Obstacle detected in front, turning the object")
                self.vel.angular.z = 0
                self.vel.linear.x = 0
                self.cmd_vel_pub.publish(self.vel)
                self.vel.angular.z = 1
                self.cmd_vel_pub.publish(self.vel)

                
            self.cmd_vel_pub.publish(self.vel)
            self.rate.sleep()


if __name__ == '__main__':
    node = maze()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
