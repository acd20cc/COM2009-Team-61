#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move

class FaceColour():

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        #self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
        #   Image, self.camera_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback) 
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.array_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 200
        crop_height = 200
        print(height)
        print(cv_img.shape)
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        #crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        crop_img = cv_img[int(height/3):int((height/2)+(height/4)), 0:width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        #blue
        blue_lower = (102, 240, 80)
        blue_upper = (107, 260, 200)
        blue = (blue_lower, blue_upper)

        #yellow
        yellow_lower = (18, 170, 50)
        yellow_upper = (30, 260, 200)
        yellow = (yellow_lower, yellow_upper)

        #green
        green_lower = (75, 150, 70)
        green_upper = (92, 260, 150)
        green = (green_lower, green_upper)

        #red
        red_lower = (-2, 190, 50)
        red_upper = (6, 260, 255)
        red = (red_lower, red_upper)

        #
        #change colour to fit whichever colour is being looked for
        #
        mask = cv2.inRange(hsv_img, blue[0], blue[1])
        #mask = cv2.inRange(hsv_img, yellow[0], yellow[1])
        #mask = cv2.inRange(hsv_img, green[0], green[1])
        #mask = cv2.inRange(hsv_img, blue[0], blue[1])


        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 20:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            elif self.move_rate == 'stop' and self.stop_counter < 20:
                print(f"MOVING to next blob of colour, scanning the area")
                if(self.array_counter == 3):
                    self.array_counter = 0
                else:
                    self.array_counter += 1
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    node = FaceColour()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
