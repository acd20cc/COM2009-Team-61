#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import numpy as np

import os

#uses camera topic and checks if colours are detected
class DetectPillar():

    def __init__(self):
        # node_name = "detect_colour"
        # rospy.init_node(node_name)

        #self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
        #   Image, self.camera_callback)
        #self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.main_colour = ""
        self.current_directory = os.path.dirname(os.path.realpath(__file__))

        self.ctrl_c = False

        self.rate = rospy.Rate(10)
        self.num_to_colour = {0:"blue", 1:"yellow", 2:"green", 3:"red"}
        self.colour_to_num = {"blue":0, "yellow":1, "green":2, "red":3}
        #dict of colour=>bool of whether a full cylinder found
        self.full_cylinder = {"blue":False, "yellow":False, "green":False, "red":False}
        #dict of cylinder information (moments, approx distance)
        self.cylinder_info = {0:[],1:[],2:[],3:[]}
        #CHANGE FOR IRL ROBOT
        self.camera_resolution = [1920, 1080]

    def shutdown_ops(self):
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
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        
        hsv_orig_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        crop_img = cv_img[int(height/3):int((height/2)+(height/4)), 0:width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        #SIMULATION COLOURS:
        #blue
        blue_lower = (115, 224, 100)
        blue_upper = (130, 255, 255)
        blue = (blue_lower, blue_upper)

        #yellow
        yellow_lower = (25, 90, 100)
        yellow_upper = (35, 260, 255)
        yellow = (yellow_lower, yellow_upper)

        #green
        green_lower = (50,105,0)
        green_upper = (65,260,255)
        green = (green_lower, green_upper)

        #red
        red_lower = (-5,130,0)
        red_upper = (7,260,255)
        red = (red_lower, red_upper)
        
        #IRL COLOURS:
        # #blue
        # blue_lower = (102, 240, 80)
        # blue_upper = (107, 260, 200)
        # blue = (blue_lower, blue_upper)

        # #yellow
        # yellow_lower = (18, 170, 50)
        # yellow_upper = (30, 260, 200)
        # yellow = (yellow_lower, yellow_upper)

        # #green
        # green_lower = (75, 150, 70)
        # green_upper = (92, 260, 150)
        # green = (green_lower, green_upper)

        # #red
        # red_lower = (-2, 190, 50)
        # red_upper = (6, 260, 255)
        # red = (red_lower, red_upper)

        #set mask ranges for each colour
        blue_mask = cv2.inRange(hsv_img, blue[0], blue[1])
        yellow_mask = cv2.inRange(hsv_img, yellow[0], yellow[1])
        green_mask = cv2.inRange(hsv_img, green[0], green[1])
        red_mask = cv2.inRange(hsv_img, red[0], red[1])

        #tries all masks on image
        img_blue =  cv2.bitwise_and(crop_img, crop_img, mask = blue_mask)
        img_yellow =  cv2.bitwise_and(crop_img, crop_img, mask = yellow_mask) 
        img_green =  cv2.bitwise_and(crop_img, crop_img, mask = green_mask) 
        img_red =  cv2.bitwise_and(crop_img, crop_img, mask = red_mask)

        #find contours in image for each masked image
        blue_contours,_ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours,_ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours,_ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_contours,_ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_array = [blue_contours, yellow_contours, green_contours, red_contours]

        masked_img_array = [img_blue, img_yellow, img_green, img_red]
        self.calc_main_colour(masked_img_array)

        #get biggest found area for the main colour contour
        biggest_area = 0
        area_cnt_arr = []
        if(self.main_colour != ""):
            for cnt in contour_array[self.colour_to_num[self.main_colour]]:
                area = cv2.contourArea(cnt)
                if(area >= biggest_area):
                    area_cnt_arr = [area, cnt]
                    biggest_area = area

            #if area is bigger than a certain amount, the robot is seeing the full cylinder
            if(area_cnt_arr[0] > 25000):
                self.full_cylinder[self.main_colour] = True
                #find and set moments for the full cylinder
                M = cv2.moments(area_cnt_arr[1])
                m0 = (M['m00'] + 1e-5)
                cx = int(M['m10']/(M['m00'] + 1e-5))
                cy = int(M['m01']/(M['m00'] + 1e-5))
                self.cylinder_info[self.colour_to_num[self.main_colour]] = [m0,cx,cy]
            else:
                self.full_cylinder[self.main_colour] = False

        #
        #for original image
        #
        #set mask ranges for each colour
        blue_mask = cv2.inRange(hsv_orig_img, blue[0], blue[1])
        yellow_mask = cv2.inRange(hsv_orig_img, yellow[0], yellow[1])
        green_mask = cv2.inRange(hsv_orig_img, green[0], green[1])
        red_mask = cv2.inRange(hsv_orig_img, red[0], red[1])

        #find contours in image for each masked image
        blue_contours,_ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours,_ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours,_ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_contours,_ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_array = [blue_contours, yellow_contours, green_contours, red_contours]

        #get biggest found area for each colours' contour
        biggest_area = 0
        colour_cnt_arr = []
        for contours in contour_array:
            area_cnt_arr = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if(area >= biggest_area):
                    area_cnt_arr = [area, cnt]
                    biggest_area = area
            colour_cnt_arr.append(area_cnt_arr)

        #loop through all colours and approximate
        col_approx_area_dict = {0:[],1:[],2:[],3:[]}
        counter = 0
        for area_cnt in colour_cnt_arr:
            colour = self.num_to_colour[counter]
            #if colour not detected (no contours for it)
            if(area_cnt == []):
                self.cylinder_info[counter] = []
                counter+=1
                continue
            #dont do distance calc if full cylinder is seen for that colour
            if(self.full_cylinder[colour]):
                continue
            #if area greater than some threshold
            if(area_cnt[0] >= 50):
                #get approximation for polygon
                approx = cv2.approxPolyDP(area_cnt[1], 0.015 * cv2.arcLength(area_cnt[1], True), True)
                #coordinates of rectangle generated by approximation
                x1,y1,w1,h1 = cv2.boundingRect(approx)
                cv2.rectangle(cv_img,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
                #coordinates of rectangle generated by actual contours
                x,y,w,h = cv2.boundingRect(area_cnt[1])
                #get average of the 2 calculations' width and calc approx distance using that
                average_width = (w+w1)/2
                approx_distance = approx_dist(average_width)

                M = cv2.moments(area_cnt[1])
                m0 = (M['m00'] + 1e-5)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #save found moments and approx distance for each colour detected in a dict to
                #be accessed and acted on outside the class
                self.cylinder_info[counter] = [m0,cx,cy,approx_dist(average_width)]

                #cv2.rectangle(hsv_orig_img,(x,y),(x+w,y+h),(0,255,0),2)
                #print(cnt_area_dict[1])
                
                counter+=1
        cv2.imshow('image', cv_img)
        cv2.waitKey(1)
            
    def calc_main_colour(self, masked_img_array):
        colour = ""
        #finds colour for which there is the most detected in image
        for i in range(4):
            for j in range(i,4):
                if(np.sum(masked_img_array[i]) > np.sum(masked_img_array[j])):
                    colour = self.num_to_colour[i]
        self.main_colour = colour

    def cylinder_detected(self):
        for i in self.full_cylinder.keys():
            if(self.full_cylinder[i]):
                return True
        return False

        def save_image(self):
            abc = 1
#approximates distance of the cylinder based on its observed width
def approx_dist(width):
    #set the base known relationship of distance to pixel width of cylinder
    base_distance = 1 #in metres
    base_pixels = 125 #in pixels
    ratio = base_pixels/width
    distance = base_distance * ratio
    return distance
