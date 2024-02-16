#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class FigureOfEight():
    def callback(self, topic_data: Odometry):
        self.node_name = "figure_eight_node"

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2) 
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main(self):
        vel_msg = Twist()

        # Set the linear and angular velocities
        vel_msg.linear.x = 0.26  # Maximum linear velocity
        vel_msg.angular.z = 2 * vel_msg.linear.x  # Adjust this value as needed for circular motion

        while not self.ctrl_c:
            # Publish the velocity message
            self.pub.publish(vel_msg)
            # Sleep to maintain a steady loop rate
            self.rate.sleep()


if __name__ == '__main__':
    node = FigureOfEight() 
    node.main()