#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np


class MazeSolver:

    def __init__(self):
        self.node_name = "maze_solver_node"
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # maximise the default velocity
        self.max_velocity = 0.26
        self.min_left = 0
        self.min_front= 0
        self.min_right = 0

        # default fron distance
        self.front_dist = 0 
        self.left_dist = 0
        self.right_dist = 0
        
        self.vel = Twist()

        # set decided path a name
        self.decision = ''

        # default for stopping the code
        self.ctrl_c = False

        # store completed locations in to set
        self.visited = set()  

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        # default function to kill the program when contrl c is entered
        self.cmd_vel_pub.publish(Twist())
        self.ctrl_c = True

    def laser_callback(self, scan_data):
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


    def move(self):
        """
        Default Maze-solve algorithm
        """
        while not self.ctrl_c:
            min_dist = self.front_dist > 0.40

            # If the distance of right side is less than 0.3 and distance front side is bigger than 0.39 turn right
            # right_distance is used to prioritise the right side from two ways.
            if self.right_dist < 0.3 and min_dist:
                print("Chose to turn Left")
                print(self.posx)
                print(self.posy)
                self.vel.linear.x = self.max_velocity 
                self.vel.angular.z = 0.9
                self.decision = "left"

            # If the distance on the right is between 0.3 and 0.4 units and min_dist is true, proceed forward.
            elif 0.3 < self.right_dist < 0.4 and min_dist:
                print("Chose to go front")
                print(self.posx)
                print(self.posy)
                self.vel.linear.x = self.max_velocity
                self.vel.angular.z = 0
                self.decision = "forward"
                # Centering logic to stay in the middle of the path
                if abs(self.left_dist - self.right_dist) > 0.15:
                    if self.left_dist > self.right_dist:
                        # Adjust to move right
                        self.vel.angular.z -= 0.1
                    else:
                        # Adjust to move left
                        self.vel.angular.z += 0.1

            # If right side is open, turn to the right side
            elif self.right_dist > 0.3 and min_dist:
                print("Chose to turn right")
                print(self.posx)
                print(self.posy)
                self.vel.linear.x = self.max_velocity
                self.vel.angular.z = -0.9
                self.decision = "right"

            # else turn 180 degree until free space is scanned.
            else:
                if self.left_dist > 0.3 and self.front_dist > 0.42:
                    print("Obstacle detected in front, turning the object to the left")
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 1.0
                    self.decision = "backward"

                elif self.right_dist < 0.3 and self.front_dist > 0.42:
                    print("Obstacle detected in front, turning the object to the right")
                    self.vel.linear.x = 0
                    self.vel.angular.z = -1.0
                    self.decision = "backward"
                else:
                    print("turning")
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 1.0



                
            

            # publish the movement
            self.cmd_vel_pub.publish(self.vel)
            self.rate.sleep()
            
    def executePath(self):
        """
        Execute the chosen movement.
        """
        # Mark forward as visited
        if self.decision == "forward":
            self.visited.add("forward")
        # Mark left as visited
        elif self.decision == "left":
            self.visited.add("left") 
        # Mark right as visited
        elif self.decision == "right":
            self.visited.add("right")  
        # Mark backward as visited
        elif self.decision == "backward":
            self.visited.add("backward")
        self.cmd_vel_pub.publish(self.vel)
        



    def main(self):
        """
        Main loop for the maze-solving algorithm.
        """
        while not self.ctrl_c:
            #execute the previous path
            decision = self.move()
            self.executePath(decision)
            self.rate.sleep()

if __name__ == '__main__':
    solver = MazeSolver()
    try:
        solver.main()
    except rospy.ROSInterruptException:
        pass
