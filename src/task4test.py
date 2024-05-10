#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from detect_colour import DetectPillar
import numpy as np

class task4test:

    def __init__(self):
        self.node_name = "t4_node"

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.detect_colour = DetectPillar()

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.detect_colour.camera_callback) 
        
        self.published = 0
        self.moving_to_coor = False
        self.turn_rate = ""
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.random_generated = False
        self.time_started = 0

        self.colour_turning_to = 0
        self.cylinder_coordinates = {"blue":[], "yellow":[], "green":[], "red":[]}
        self.cylinders_found = [] #list of found cylinders
        
        self.odom_data = Odometry()
        self.vel = Twist()
        self.pose_stamp = PoseStamped()
        self.ctrl_c = False

        self.m00 = 0
        self.m00_min = 10000
        
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.cmd_vel_pub.publish(Twist())
        self.move_base_pub.publish(PoseStamped())
        self.ctrl_c = True

    def odom_callback(self, odom_data):
        self.odom_data = odom_data

    def scan_callback(self, scan_data):

        right_arc = scan_data.ranges[-90:]
        left_arc = scan_data.ranges[0:91]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.right_dist = np.array(front_arc[120:150]).min()
        self.front_dist= np.array(front_arc[70:110]).min()
        self.left_dist = np.array(front_arc[30:60]).min()
        
    def main(self):
        self.pose_stamp.header.frame_id = 'map'
        while not self.ctrl_c:
            #if cylinder detected and colour not found before
            if(self.detect_colour.cylinder_detected and not(self.detect_colour.main_colour in self.cylinders_found)):
                print("Cylinder of colour: " + self.detect_colour.main_colour + " found")
                self.cylinders_found.append(self.detect_colour.main_colour)
                self.moving_to_coor = False
                self.published = 0
            #decides turn rate based on moments stored
            self.decide_turn_rate()
            if(self.turn_rate == "fast"):
                self.vel.linear.x = 0.0
                self.vel.angular.z = (self.turn_vel_fast)
            if(self.turn_rate == "slow"):
                self.vel.linear.x = 0.0
                self.vel.angular.z = (self.turn_vel_slow)
            if(self.turn_rate == "stop"):
                self.vel.linear.x = 0.0
                self.vel.angular.z = (0.0)
                self.est_coordinate()
            self.cmd_vel_pub.publish(self.vel)
            for i in range(4):
                #if it has coordinate and not seen that colour, then go to it
                if (self.cylinder_coordinates[i] != []) and (not (self.detect_colour.num_to_colour[i] in self.cylinders_found)):
                    self.pose_stamp.pose.position.x = self.cylinder_coordinates[i][0]
                    self.pose_stamp.pose.position.y = self.cylinder_coordinates[i][1]
                    self.pose_stamp.pose.orientation.w = 1.0
                    if(self.published < 10):
                        self.move_base_pub.publish(self.pose_stamp)
                        self.published += 1
                    self.moving_to_coor = True
                    break
            if ((self.turn_rate == "") and (self.moving_to_coor == False)):
                self.move_to_random_coordinate()
                    
            self.rate.sleep()

    #generates random coordinates and makes robot move to them
    def move_to_random_coordinate(self):
        time_now = rospy.get_rostime().secs
        if(not self.random_generated):
            self.time_started = rospy.get_rostime().secs
            self.pose_stamp.pose.position.x = (np.random.random_sample() * 4) -2
            self.pose_stamp.pose.position.y = (np.random.random_sample() * 4) -2
            self.pose_stamp.pose.orientation.w = 1.0
        if(self.published < 10):
            self.move_base_pub.publish(self.pose_stamp)
            self.random_generated = True
            self.published += 1
        #will generate new random coordinates after a certain time has passed
        if(self.time_started - time_now > 60):
            self.random_generated = False

    def decide_turn_rate(self):
        self.turn_rate = ""
        for i in range(4):
            #no coordinates for that colour and colour not fully seen before
            if((self.cylinder_coordinates[i] == []) and (not (self.detect_colour.num_to_colour[i] in self.cylinders_found))):
                #there is recorded estimated distance information for that cylinder colour
                if(not (self.detect_colour.cylinder_info[i] == [])):
                    self.colour_turning_to = i
                    self.m00 = self.detect_colour.cylinder_info[i][0]
                    cy = self.detect_colour.cylinder_info[i][2]
                    if self.m00 > self.m00_min:
                        # blob detected
                        if cy >= 420-60 and cy <= 420+60:
                            self.turn_rate = 'stop'
                        else:
                            self.turn_rate = 'slow'
                    else:
                        self.turn_rate = 'fast'
                    break
    
    def est_coordinate(self):
        #take approx dist num for that colour
        distance = self.detect_colour.cylinder_info[self.colour_turning_to][3]
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orientation = self.odom_data.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, 
                                                    orientation.z, orientation.w], 'sxyz')
        x_dist = distance * np.sin(np.pi - yaw)
        y_dist = distance * np.cos(np.pi - yaw)
        coordinate = [robot_x+x_dist, robot_y+y_dist]
        self.cylinder_coordinates[self.colour_turning_to] = coordinate

        



if __name__ == '__main__':
    node = task4test()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
