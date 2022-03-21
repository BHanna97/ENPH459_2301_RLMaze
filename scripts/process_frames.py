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
      cv_image =  bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)
    #dt=datetime.now()
    #rospy.loginfo(dt.microsecond/1000)
    #cv2.imshow('cv_img',cv_image)
    #print("Image recieved")
    get_command(cv_image)

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send an error value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command(raw_image):
    platform_image=crop_platform(raw_image)	
    ball_found, ball_loc=track_ball(platform_image)
    return False

#Takes a raw image of the scene and returns a cropped image of only a rectangular platform
def crop_platform(raw_image):
    edges=cv2.Canny(raw_image, 100, 200, apertureSize=3, L2gradient = True) #doesn't seem very effective
    #cv2.imshow('lines',edges)
    #cv2.waitKey(0)
    return(raw_image)

#Takes in cropped and straightened image of platform and returns ball state
#dev note: after implementing quite a bit of framework to use a tracking algorithm
#rather than finding the ball from scratch each frame, I discovered that most algorithms
#are mysteriously not supported in opencv 4.5.5-dev. MIL, which I am using has been 
#reported to have issues recording failures, which I am experiencing as well
#The other options are DaSiamRPN and GOTURN, both of which require extra setup.
def track_ball(platform_image):
    global ball_on_platform
    if not ball_on_platform: #The ball was not found in the previous frame
        ball_found,box = find_ball(platform_image)
        if ball_found:
            #ball_on_platform=True
            tracker.init(platform_image,box)
    else:
        ball_found, box = tracker.update(platform_image)
        if not ball_found:
            ball_on_platform=False
            rospy.loginfo("ball lost!")

    (x,y,w,h)= [int(v) for v in box]
    #debug
    tracked=cv2.rectangle(platform_image, (x,y), (x+w, y+h), (0,255,0), 2)
    cv2.imshow('box',tracked)
    cv2.waitKey(1)

    if ball_found:
        return True, [x+w/2,y+h/2] 
    else:
        return False, None

#TODO: try to find the ball, and return either True and
#a bounding box as an array of the top left, then top right corner coordinates
#or return False and None
def find_ball(platform_image):
    threshold = 70 #TODO:tune to real environment, or implement something more sophisticated like Otsu
    #may want to gaussian blur here
    _, thresh = cv2.threshold(platform_image, threshold, 255, cv2.THRESH_BINARY_INV)
    erosion_kernel = np.ones((5,5), np.uint8)
    thresh_eroded = cv2.erode(thresh,erosion_kernel, iterations = 3) #TODO: tune
    cv2.imshow('raw',platform_image)
    cv2.imshow('threshold', thresh)
    cv2.imshow('thresh_eroded', thresh_eroded)
    cv2.waitKey(1)

    #print(np.sum(thresh_eroded))
    if (np.sum(thresh_eroded) > 500): #TODO:Tune
        rospy.loginfo("ball found!")
    #    contours=cv2.findContours(thresh_eroded,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return(True, cv2.boundingRect(thresh_eroded))
    else:
        return False, [-1,-1,-1,-1]
def listener():
    
    rospy.init_node('frame_processor', anonymous=True)
    
    rospy.Subscriber('picam_image', Image, callback)
    
    angle_pub=rospy.Publisher('target_platform_angle', Quaternion, queue_size=1)
    state_pub=rospy.Publisher('ball_on_platform', Bool, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
    tracker = cv2.TrackerMIL_create()
    global ball_on_platform
    ball_on_platform = False
    listener()
