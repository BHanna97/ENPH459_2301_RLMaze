#!/usr/bin/env python2

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
    cv2.imwrite('test.jpg', cv_image)
    get_command(cv_image)

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send an error value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command(raw_image):
    platform_image=crop_platform(raw_image)	
    ball_found, ball_loc=track_ball(platform_image)
    return False

#Takes in cropped and straightened image of platform and returns ball state
#dev note: after implementing quite a bit of framework to use a tracking algorithm
#rather than finding the ball from scratch each frame, I discovered that most algorithms
#are mysteriously not supported in opencv 4.5.5-dev. MIL, which I am using has been 
#reported to have issues recording failures, which I am experiencing as well
#The other options are DaSiamRPN and GOTURN, both of which require extra setup.
def track_ball(platform_image):
    global ball_on_platform
    global ball_loc
    if not ball_on_platform: #The ball was not found in the previous frame
        ball_found,ball_loc = identify_ball(platform_image)
        if ball_found:
            ball_on_platform=True
            rospy.loginfo('ball found!')
    else: #we know where the ball is, we don't need to search the whole platform
        search_rad=100
        (x,y,w,h)= [int(v) for v in ball_loc]
        local_image = platform_image[y-search_rad:y+search_rad,x-search_rad:x+search_rad]
        ball_found, ball_loc = identify_ball(local_image)
        ball_loc = [ball_loc[0]+x-search_rad,ball_loc[1]+y-search_rad,ball_loc[2],ball_loc[3]]
        if not ball_found:
            #TODO: add some inertia here so that one failed frame doesn't trigger a reset
            ball_on_platform=False
            rospy.loginfo("ball lost!")

    (x,y,w,h)= [int(v) for v in ball_loc]
    #debug
    tracked=cv2.rectangle(platform_image, (x,y), (x+w, y+h), (0,255,0), 2)
    cv2.imshow('box',tracked)
    cv2.waitKey(1)

    if ball_found:
        return True, [x+w/2,y+h/2] 
    else:
        return False, None

def identify_ball(image):
    #may want to gaussian blur here
    #TODO:tune
    _,thresh_high = cv2.threshold(image, 215,255, cv2.THRESH_BINARY)
    _,thresh_low = cv2.threshold(image, 125,255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((5,5), np.uint8)
    try:
        high_dilated = cv2.dilate(thresh_high,kernel, iterations = 1) #TODO: tune
        low_dilated = cv2.dilate(thresh_low,kernel, iterations = 1) #TODO: tune
        ball_mask=cv2.bitwise_and(high_dilated,low_dilated)

        cv2.imshow('brightest', high_dilated)
        cv2.imshow('undilated', thresh_high)
        cv2.imshow('darkest', thresh_low)
    except:
        rospy.loginfo('thresh blanked')
        ball_mask=0


    cv2.imshow('ball_mask', ball_mask)
    cv2.waitKey(1)

    #print(np.sum(thresh_eroded))
    if (np.sum(ball_mask) > 2000000): #TODO:Tune
    #    contours=cv2.findContours(thresh_eroded,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return(True, cv2.boundingRect(ball_mask))
    else:
        return False, [-1,-1,-1,-1]

def crop_platform(image):
    global maze_mask
    global prev_marker_centroids
    #Bounding regions of tape, as x,y,w,h, starting at the top left and going clockwise
    #found experimentally
    #TODO: tune to smaller bounding boxes

    marker_boxes=[
    [350,0,150,200],
    [550,200,200,150],
    [0,350,100,150],
    [200,550,150,150],
    ]
    marker_centroids=[[0,0],[0,0],[0,0],[0,0]]
    for i in range(0,4):
        marker_centroids[i]=get_tape_centroid(image, marker_boxes[i])
        if marker_centroids[i]==None:
            marker_centroids[i]=prev_marker_centroids[i]
    prev_marker_centroids=marker_centroids 

    #test_cols=((255,0,0),(0,255,0),(0,0,255),(255,255,255))
    #for i in range(0,4):
    #    cv2.circle(image, marker_centroids[i],2, test_cols[i], -1) 
    #    cv2.rectangle(image, marker_boxes[i],test_cols[i] ,1)
    #    cv2.imshow('image', image)
    #    cv2.waitKey(1)
    
    image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ptransform = cv2.getPerspectiveTransform(np.float32(marker_centroids),np.float32([[474,24],[660,300],[37,319],[215,535]]))
    straight = cv2.warpPerspective(image_bw, ptransform, (678,577), flags = cv2.INTER_LINEAR)
    straight_clean = cv2.bitwise_and(straight,straight,mask=maze_mask)
    #cv2.imshow('straightened', straight_clean)
    
    return straight_clean

def crop(img, bb):
    return  img[bb[1]:bb[1]+bb[3],bb[0]:bb[0]+bb[2]]

def get_tape_centroid(image,bb):
    local_image = crop(image,bb)
    img_hsv=cv2.cvtColor(local_image, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(img_hsv,(95,175,212),(130,255,255))
    M = cv2.moments(thresh)
    #cv2.imshow('local_image',local_image)
    #cv2.imshow('thresh', thresh)
    cv2.waitKey(1)
    if M["m00"]!=0: 
        cent_local =  [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])]
        return [bb[0]+cent_local[0],bb[1]+cent_local[1]]
    else:
        return None

def listener():
    
    rospy.init_node('frame_processor', anonymous=True)
    
    rospy.Subscriber('picam_image', Image, callback)
    
    angle_pub=rospy.Publisher('target_platform_angle', Quaternion, queue_size=1)
    state_pub=rospy.Publisher('ball_on_platform', Bool, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
    #tracker = cv2.TrackerMIL_create()
    global ball_on_platform
    ball_on_platform = False
    global maze_mask
    maze_mask=cv2.imread('/home/pi/ros_catkin_ws/src/project_theseus/scripts/straightened_platform_mask.jpg',cv2.IMREAD_GRAYSCALE)

    listener()
