#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from detect_colour import DetectPillar
import numpy as np

class task4test:

    def __init__(self, target_colour):
        self.node_name = "t4_node"

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.detect_colour = DetectPillar()

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.move_base_simple_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        #self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.detect_colour.camera_callback) 
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.detect_colour.camera_callback)
        
        self.published = 0
        self.moving_to_coor = False
        self.turn_dir = 1
        self.turn_rate = ""
        self.turn_vel_fast = 0.2
        self.turn_vel_slow = 0.05
        self.random_generated = False
        self.time_started = 0

        self.time_now = time_now = rospy.get_rostime().secs

        self.colour_turning_to = 0
        self.cylinder_coordinates = {"blue":[], "yellow":[], "green":[], "red":[]}
        self.cylinders_found = [] #list of found cylinders
        self.colour_goal_id = {"random":[],"blue":[], "yellow":[], "green":[], "red":[]}
        
        self.odom_data = Odometry()
        self.vel = Twist()
        self.pose_stamp = PoseStamped()
        self.goal_status_arr = GoalStatusArray()
        self.goal_status = GoalStatus()
        self.goal_id = GoalID()
        self.ctrl_c = False
        self.camera_width = 1920
        self.camera_height = 1080

        self.m00 = 0
        self.m00_min = 1000

        self.target_colour = target_colour
        
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def shutdown_hook(self):
        self.cmd_vel_pub.publish(Twist())
        self.move_base_simple_pub.publish(PoseStamped())
        self.ctrl_c = True
    
    def goal_status_callback(self, goal_status_data):
        self.goal_status_arr = goal_status_data

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
        rospy.loginfo("TASK 4 BEACON: The target is {colour}.")
        while not self.ctrl_c:
            #if cylinder detected and colour not found before
            if(self.detect_colour.cylinder_detected() and not(self.detect_colour.main_colour in self.cylinders_found)):
                print("Cylinder of colour: " + self.detect_colour.main_colour + " found")
                self.cylinders_found.append(self.detect_colour.main_colour)
                self.moving_to_coor = False
            # #decides turn rate based on moments stored
            # self.decide_turn_rate()
            # if(self.turn_rate == "fast"):
            #     self.vel.linear.x = 0.0
            #     self.vel.angular.z = self.turn_vel_fast * self.turn_dir
            #     self.cmd_vel_pub.publish(self.vel)
            # if(self.turn_rate == "slow"):
            #     self.vel.linear.x = 0.0
            #     self.vel.angular.z = self.turn_vel_slow * self.turn_dir
            #     self.cmd_vel_pub.publish(self.vel)
            # if(self.turn_rate == "stop"):
            #     for i in range(30):
            #         self.rate.sleep()
            #     self.vel.linear.x = 0.0
            #     self.vel.angular.z = 0.0
            #     self.cmd_vel_pub.publish(self.vel)
            #print(self.vel.linear.x, " ", self.vel.angular.z)
            difference = rospy.get_rostime().secs - self.time_now
            #print(self.turn_rate)
            if ((self.turn_rate == "") and (self.moving_to_coor == False) 
                and ((difference > 20) or not self.processed_goal("random"))):
                self.move_to_random_coordinate(difference)
                    
            self.rate.sleep()

    #returns true if move_base goal has been processed for that colour
    def processed_goal(self, colour):
        if(self.goal_status_arr.status_list == []):
            return False
        self.colour_goal_id[colour] = self.goal_status.goal_id
        self.goal_status = self.goal_status_arr.status_list[0]
        self.goal_id = self.goal_status.goal_id
        if(self.goal_status.status < 4):
            return True
        return False

    #cancels the inputted goal
    def cancel_goal(self, goal_id):
        self.move_base_cancel.publish(goal_id)

    #generates random coordinates and makes robot move to them
    def move_to_random_coordinate(self, difference):
        if(not self.random_generated):
            self.generate_rand_coor()
        if(not self.processed_goal("random")):
            print("published rand")
            self.move_base_simple_pub.publish(self.pose_stamp)
            self.random_generated = True
        #will generate new random coordinates after a certain time has passed
        if(difference > 20):
            print("published new rand")
            self.generate_rand_coor()
            self.move_base_simple_pub.publish(self.pose_stamp)
            self.time_now = rospy.get_rostime().secs

    def generate_rand_coor(self):
        self.time_started = rospy.get_rostime().secs
        self.pose_stamp.pose.position.x = (np.random.random_sample() * 3.8) - 1.9
        self.pose_stamp.pose.position.y = (np.random.random_sample() * 3.8) - 1.9
        self.pose_stamp.pose.orientation.w = 1.0

    def decide_turn_rate(self):
        self.turn_rate = ""
        for i in range(4):
            #no coordinates for that colour and colour not fully seen before
            c = self.detect_colour.num_to_colour[i]
            if((self.cylinder_coordinates[c] == []) and (not (c in self.cylinders_found))):
                #there is recorded estimated distance information for that cylinder colour
                if(not (self.detect_colour.cylinder_info[i] == [])):
                    #cancel current goal to look at new cylinder
                    #if(self.processed_goal(c)):
                        #print("cancelled goal")
                        #self.cancel_goal(self.goal_id)
                    self.colour_turning_to = i
                    self.m00 = self.detect_colour.cylinder_info[i][0]
                    cx = self.detect_colour.cylinder_info[i][1]
                    cy = self.detect_colour.cylinder_info[i][2]
                    if self.m00 > self.m00_min:
                        half_width = self.camera_width/2
                        if cx < half_width:
                            self.turn_dir = 1
                        else:
                            self.turn_dir = -1
                        # blob detected
                        if cx >= half_width-60 and cx <= half_width+60:
                            self.turn_rate = 'stop'
                        elif cx >= half_width-240 and cx <= half_width+240:
                            self.turn_rate = 'slow'
                        else:
                            self.turn_rate = 'fast'
                    break



if __name__ == '__main__':
    target_colour = rospy.get_param('~target_colour', 'red')
    node = task4test(target_colour)
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
