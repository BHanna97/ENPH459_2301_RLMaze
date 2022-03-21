#!/usr/bin/env python

import cv2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from datetime import datetime

def callback(data):
    try:
      cv_image =  bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    #dt=datetime.now()
    #rospy.loginfo(dt.microsecond/1000)
    #cv2.imshow('cv_img',cv_image)
    #print("Image recieved")
    get_command(cv_image)

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send a null(?) value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command(raw_image):
    platform_image=crop_platform(raw_image)	
    ball_found, ball_loc=track_ball(platform_image)
    return False

#Takes a raw image of the scene and returns a cropped image of only a rectangular platform
def crop_platform(raw_image):
    edges=cv2.Canny(raw_image, 100, 200, apertureSize=3, L2gradient = True)
    cv2.imshow('lines',edges)
    cv2.waitKey(0)
    return(raw_image)

#takes in cropped and straightened image of platform and returns ball state
#Loosely based on https://pyimagesearch.com/2018/07/30/opencv-object-tracking
def track_ball(platform_image):
    if not ball_on_platform: #The ball was not found in the previous frame
        ball_found,box = find_ball(platform_image)
        if ball_found:
            cv2.TrackerKCF.init(platform_image,box)
    else:
        ball_found, box = cv2.TrackerKCF.update(platform_image)
        if not ball_found:
            ball_on_platform=False
            rospy.loginfo("ball lost!")
    if ball_found:
        return True, [np.mean(box[1],box[3]), np.mean(box[2],box[4])] 
    else:
        return False, None

#TODO: try to find the ball, and return either True and
#a bounding box as an array of the top left, then top right corner coordinates
#or return False and None
def find_ball(platform_image):
    return false, None
def listener():
    
    rospy.init_node('frame_processor', anonymous=True)
    
    rospy.Subscriber('picam_image', Image, callback)
    
    angle_pub=rospy.Publisher('target_platform_angle', Quaternion, queue_size=1)
    state_pub=rospy.Publisher('ball_on_platform', Bool, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
   # tracker = cv2.TrackerKCF_create
    ball_on_platform = False
    listener()
