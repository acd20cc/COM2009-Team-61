#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class maze:

    def __init__(self):
        self.node_name = "t4_node"

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.min_left = 0
        self.min_front= 0
        self.min_right = 0
        self.counter = 0

        self.published = 0

        self.front_dist = 0 
        self.left_dist = 0
        self.right_dist = 0
        
        self.vel = Twist()
        self.pose_stamp = PoseStamped()
        self.ctrl_c = False
        

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.cmd_vel_pub.publish(Twist())
        self.move_base_pub.publish(PoseStamped())
        self.ctrl_c = True

    def odom_callback(self, odom_data):
        self.counter +=1
        if((self.counter % 100) == 0):
            #print(odom_data)
            abc = 1

    def scan_callback(self, scan_data):

        right_arc = scan_data.ranges[-90:]
        left_arc = scan_data.ranges[0:91]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.right_dist = np.array(front_arc[120:150]).min()
        self.front_dist= np.array(front_arc[70:110]).min()
        self.left_dist = np.array(front_arc[30:60]).min()

        
    def main(self):
        """
        Main loop for stuff.
        """
        while not self.ctrl_c:
            #self.cmd_vel_pub.publish(self.vel)
            #print(self.pose_stamp)
            self.pose_stamp.header.frame_id = 'map'
            self.pose_stamp.pose.position.x = -1.4
            self.pose_stamp.pose.position.y = -0.3
            self.pose_stamp.pose.orientation.w = 1.0
            if(self.published < 10):
                self.move_base_pub.publish(self.pose_stamp)
                self.published += 1
            self.rate.sleep()
                


if __name__ == '__main__':
    node = maze()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
