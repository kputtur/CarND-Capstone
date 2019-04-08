import cv2
import rospy
from styx_msgs.msg import TrafficLight
import numpy as np

"""
Reference implementation from
https://github.com/sol-prog/OpenCV-red-circle-detection/blob/master/circle_detect.cpp
"""

class OCVModel():
    def __init__(self):
        self.img_counter = 100
        #rospy.logwarn("Entering initalization")
   
    def predict(self, image):
        """
        image: cv2.Image(BGR)
        """
        #rospy.logwarn("Entering ...")
        self.img_counter = self.img_counter+1
        
        #MedianBlur the image to handle noise
        img_bgr = cv2.medianBlur(image, 3)
        
        #Convert input image to HSV
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV) 

        realworld_red_yellow_range = cv2.inRange(img_hsv,np.array([10, 40, 250]), np.array([31, 150, 255]))
    	sim_upper_red_range = cv2.inRange(img_hsv,np.array([160, 40, 200]), np.array([180, 255, 255]))
    	sim_lower_red_range = cv2.inRange(img_hsv,np.array([0, 160, 200]), np.array([9, 255, 255]))
    	sim_yellow_range = cv2.inRange(img_hsv,np.array([29, 200, 242]), np.array([31, 255, 255]))

    	#combine above images
    	red_hue_image = cv2.addWeighted(realworld_red_yellow_range, 1.0, sim_upper_red_range, 1.0, 0.0)
    	red_hue_image = cv2.addWeighted(red_hue_image, 1.0, sim_lower_red_range, 1.0, 0.0)
    	red_hue_image = cv2.addWeighted(red_hue_image, 1.0, sim_yellow_range, 1.0, 0.0) # for simulator

	#blur the image to avoid false positives
    	blurred_image = cv2.GaussianBlur(red_hue_image,(7,7),0)
    
    	# use HoughCircles to detect circles
    	circles = cv2.HoughCircles(blurred_image,cv2.HOUGH_GRADIENT,1,100, param1=50,param2=15,minRadius=4,maxRadius=40)
		
    	# Loop through all detected circles and detect the filled circle
    	# Find the filled ratio, if it is above filled ratio, we found match.
    
    	blurred_image_bgr = cv2.cvtColor(blurred_image,cv2.COLOR_GRAY2BGR)

    	img_with_circle = blurred_image_bgr

    	prediction = TrafficLight.UNKNOWN
    
    	if circles is not None:
        	circle_number = 0
        	for current_circle in circles[0,:]:
                #For each circle check the filled ratio
                    circle_number = circle_number + 1
            	    center_x = int(current_circle[0])
            	    center_y = int(current_circle[1])
            	    radius = int(current_circle[2])
            	    new_radius = radius + 1
            	    grid = cv2.inRange(blurred_image[center_y-new_radius:center_y+new_radius,center_x-new_radius:center_x+new_radius],230,255)
            	    area = new_radius*new_radius*4
            	    occupied = cv2.countNonZero(grid)
            	    ratio = occupied/float(area)
         	
                if ratio > 0.35:
                    prediction = TrafficLight.RED
       
        #rospy.logwarn("prediction %s", prediction) 
    	return prediction    
