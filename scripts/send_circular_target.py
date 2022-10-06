#!/usr/bin/env python

#modified from https://www.geeksforgeeks.org/displaying-the-coordinates-of-the-points-clicked-on-the-image-using-python-opencv/

import cv2

import rospy
from geometry_msgs.msg import Point
import time
import numpy as np
# function to display the coordinates of
# of the points clicked on the image

 
 
# driver function
if __name__=="__main__":

    pub=rospy.Publisher('target_point', Point, queue_size=1)
    rospy.init_node('target_transmitter', anonymous=True)
    
    centre = np.array([357,311])
    radius = 200
    t = 0

    try:
        while True:
            x=centre[0] + radius * np.sin(t)
            y=centre[1] + radius * np.cos(t)
     
            pub.publish(Point(x,y,None))
            t += 0.06
            time.sleep(0.5) 
    except(KeyboardInterrupt):
         print("exiting")
