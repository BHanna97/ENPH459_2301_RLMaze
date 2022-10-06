#!/usr/bin/env python

#modified from https://www.geeksforgeeks.org/displaying-the-coordinates-of-the-points-clicked-on-the-image-using-python-opencv/

import cv2

import rospy
from geometry_msgs.msg import Point
# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
        pub.publish(Point(x,y,None))
 
 
# driver function
if __name__=="__main__":
    image_name = 'Select a target!' 
    img = cv2.imread('/home/pi/ros_catkin_ws/src/project_theseus/scripts/straightened_platform_ref.jpg', 1)
    cv2.imshow(image_name, img)
    cv2.setMouseCallback(image_name, click_event)

    pub=rospy.Publisher('target_point', Point, queue_size=1)
    rospy.init_node('target_transmitter', anonymous=True)
 
    # wait for a key to be pressed to exit
    cv2.waitKey(0)
 
    # close the window
    cv2.destroyAllWindows()
