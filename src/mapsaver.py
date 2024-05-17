#!/usr/bin/env python3

import rospy
from subprocess import call
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import os

class mapSaver():
    def __init__(self):
        rospy.init_node('map_saver_seconds', anonymous=True)
        self.cvbridge_interface = CvBridge()
        # Set up a timer to call save_map every 20 seconds
        # self.save_map()
        # Call save_map function for 20 seconds
        self.timer = rospy.Timer(rospy.Duration(20), self.save_map)
        self.current_directory = os.path.dirname(os.path.realpath(__file__))
        self.count = 0

    def save_map(self, event):

        # Define the filename for the map files using a count.
        filename = f"task4_map"
        parent_directory = os.path.dirname(self.current_directory)
        file = parent_directory + "/maps/" + filename
        print(file)

        # Shell command to save the map using rosrun. 
        command = f"rosrun map_server map_saver -f {file}"  
        
        # Execute the command in the shell.
        call(command.split())  
        
        # Log information about the saved files.
        rospy.loginfo(f"Map saved as {filename}.yaml and {filename}.pgm")  
        
        # Increment the map count after saving a map.
        self.count += 1  

if __name__ == '__main__':
    try:
        map_saver = mapSaver()

        # Keeps your node from exiting until the node has been shutdown
        rospy.spin()  
    except rospy.ROSInterruptException:
        rospy.loginfo("Map saver node shutdown requested.")
