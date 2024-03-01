#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

class FigureEight:
    def __init__(self):
        rospy.init_node('figure_eight_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10Hz
        self.rate_counter = 0
        self.ctrl_c = False
        self.current_yaw = 0.0
        self.initial_yaw = None
        self.last_yaw = None
        # Track total yaw change (circumfernce) to determine when a circle is complete
        self.total_yaw_change = 0.0  
        self.state = 'anticlockwise'

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.last_yaw = yaw
        else:
            yaw_change = self.calculate_yaw_change(self.last_yaw, yaw)
            self.total_yaw_change += yaw_change
            self.last_yaw = yaw

    def calculate_yaw_change(self, last_yaw, current_yaw):
        # Calculate the smallest angle difference to ensure it completes the circumfernce path 
        difference = current_yaw - last_yaw
        while difference > math.pi:
            difference -= 2 * math.pi
        while difference < -math.pi:
            difference += 2 * math.pi
        return difference

    def calculate_velocity(self, state):
        vel_msg = Twist()
        if state in ['anticlockwise', 'clockwise']:
            vel_msg.linear.x = 0.115
            # Calculated angular velocity for 1m diameter circle
            if state == 'anticlockwise':
              vel_msg.angular.z = 0.23
            else:
                vel_msg.angular.z = -0.23
        else:  # state == 'stop'
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        return vel_msg


    def shutdownhook(self):
        # Stop the robot
        self.ctrl_c = True
        self.pub.publish(Twist())  

    def main(self):
        rospy.on_shutdown(self.shutdownhook)
        while not rospy.is_shutdown() and not self.ctrl_c:
            vel_msg = self.calculate_velocity(self.state)
            self.rate_counter += 1
            #so info is printed at a rate of 1hz (machine is running at 10hz)
            if(self.rate_counter % 10 == 0):
                #prints speed and yaw values
                print("x= ", vel_msg.linear.x, "m, y=", vel_msg.linear.y, "m, yaw=", self.last_yaw, "degrees.")
            self.pub.publish(vel_msg)
            
            if self.state == 'anticlockwise' and self.total_yaw_change >= 2 * math.pi:
                self.state = 'clockwise'
                # Reset yaw change counter
                self.total_yaw_change = 0 
            elif self.state == 'clockwise' and self.total_yaw_change <= -2 * math.pi + 0.1:
                self.state = 'stop'

            self.rate.sleep()

if __name__ == '__main__':
    figure_eight_node = FigureEight()
    figure_eight_node.main()
