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
    cv2.imwrite('test_hsv.jpg',cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV))
    cv2.imwrite('test.jpg', cv_image)
    get_command(cv_image)

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send an error value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command(raw_image):
    platform_image=crop_platform(raw_image)	
   # ball_found, ball_loc=track_ball(platform_image)
    return False

#Takes a raw image of the scene and returns a cropped image of only a rectangular platform
def crop_platform(raw_image):
    #edges=cv2.Canny(raw_image, 100, 200, apertureSize=3, L2gradient = True) #doesn't seem very effective
    #cv2.imshow('lines',edges)
    #cv2.waitKey(1)
    identify_tape(raw_image)
    #M = cv2.getPerspectiveTransform(np.float32([[0,442],[250,693],[650,271],[453,22]]),np.float32([[0,250],[0,50],[300,50],[300,250]]))
    #straight = cv2.warpPerspective(raw_image, M, (300,300), flags = cv2.INTER_LINEAR)
    #cv2.imshow('straightened', straight)
    return(raw_image)

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
        (x,y,_,_)= [int(v) for v in ball_loc]
        local_image = platform_image #h[y-40:y+40,x-40:x+40]
        ball_found, box = identify_ball(local_image)
        if not ball_found:
            #TODO: add some inertia here so that one failed frame doesn't trigger a reset
            ball_on_platform=False
            rospy.loginfo("ball lost!")

    (x,y,w,h)= [int(v) for v in ball_loc]
    #debug
    tracked=cv2.rectangle(platform_image, (x,y), (x+w, y+h), (0,255,0), 2)
    cv2.imshow('box',tracked)
    cv2.imshow('raw',platform_image)
    cv2.waitKey(1)

    if ball_found:
        return True, [x+w/2,y+h/2] 
    else:
        return False, None

def identify_ball(image):
    img_bw=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #may want to gaussian blur here
    #TODO:tune
    thresh_high = cv2.thresh(img_bw, 50,255, cv2.THRESH_BINARY)
    thresh_low = cv2.thresh(img_bw, 200,255, cv2.THRESH_BINARY_INV)
    erosion_kernel = np.ones((5,5), np.uint8)
    thresh_eroded = cv2.erode(thresh,erosion_kernel, iterations = 3) #TODO: tune
    cv2.imshow('threshold', thresh)
    cv2.imshow('hsv',img_hsv)
    cv2.imshow('thresh_eroded', thresh_eroded)
    cv2.waitKey(1)

    #print(np.sum(thresh_eroded))
    if (np.sum(thresh_eroded) > 500): #TODO:Tune
    #    contours=cv2.findContours(thresh_eroded,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return(True, cv2.boundingRect(thresh_eroded))
    else:
        return False, [-1,-1,-1,-1]

def identify_tape(image):
    #Bounding regions of tape, as x,y,w,h, found experimentally
    #TODO: tune to smaller bounding boxes
    topl=[0,350,100,150]
    topr=[350,0,150,200]
    botl=[200,550,150,150]
    botr=[550,200,200,150]
    topl_c=get_tape_centroid(image, topl)
    topr_c=get_tape_centroid(image, topr)
    botl_c=get_tape_centroid(image, botl)
    botr_c=get_tape_centroid(image, botr)
    
    dots = cv2.circle(image, topl_c,3, (0,0,255), -1) 
    dots = cv2.circle(dots, topr_c,3, (0,255,255), -1) 
    dots = cv2.circle(dots, botl_c,3, (255,0,255), -1) 
    dots = cv2.circle(dots, botr_c,3, (255,255,255), -1) 
    bbs=cv2.rectangle(dots,topl, (0,0,255),1)
    bbs=cv2.rectangle(dots,topr, (0,255,255),1)
    bbs=cv2.rectangle(dots,botl, (255,0,255),1)
    bbs=cv2.rectangle(dots,botr, (255,255,255),1)

    cv2.imshow('dots', dots)
    cv2.waitKey(1)

    ptransform = cv2.getPerspectiveTransform(np.float32([topl_c,botl_c,botr_c,topr_c]),np.float32([[37,319],[215,535],[660,300],[474,24]]))
    straight = cv2.warpPerspective(image, ptransform, (678,577), flags = cv2.INTER_LINEAR)
    cv2.imshow('straightened', straight)

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
    cent_local =  [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])]
    
    return [bb[0]+cent_local[0],bb[1]+cent_local[1]]

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
    listener()
