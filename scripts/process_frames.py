#!/usr/bin/env python2

import cv2

import Stewart

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import time
from collections import deque
import serial

def img_callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    #dt=datetime.now()
    #rospy.loginfo(dt.microsecond/1000)
    #cv2.imshow('cv_img',cv_image)
    #print("Image recieved")
    #cv2.imwrite('test.jpg', cv_image)
    #start_time = time.time()
    get_command(cv_image)
    #rospy.loginfo("processed frame in " + str(time.time() - start_time))

def target_callback(point):
    global target_point
    target_point = np.array([point.x, point.y], np.single)
    rospy.loginfo("target aquired!")

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send an error value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command(raw_image):
    global past_errs
    global target_point
    global rotation_matrix
    global nlostframes
    platform_image = crop_platform(raw_image)	
    ball_found, ball_loc = track_ball(platform_image)

    if nlostframes < 30: #global variable set by track_ball
    
        if ball_found:
            past_errs.append((target_point-ball_loc))
        #rospy.loginfo(past_errs)
        #rospy.loginfo(past_errs[-1])
        p_err=past_errs[-1]
        i_err=sum(past_errs)
        d_err=np.array([0,0], np.single)
        last_err=None
        for err in past_errs:
            if last_err is not None:
                d_err += d_err-last_err
            last_err=err
        p_gain = np.array([0.008,0.008], dtype=np.single)
        d_gain = np.array([0.0003,0.0003], dtype=np.single)
        i_gain = np.array([0.00015,0.00015], dtype=np.single) 
        #rospy.loginfo("proportional error:" +str(p_err))
        #rospy.loginfo("target:" + str(target_point))
        platform_angle = p_gain*p_err + d_gain*d_err + i_gain*i_err
        #rospy.loginfo(platform_angle)
        platform_angle = [platform_angle[0],-platform_angle[1]]
        rotated_angles = np.matmul(rotation_matrix,platform_angle)
        #rospy.loginfo("rotated angles:" + str(rotated_angles))
        servo_angles=theseus.orient(rotated_angles[0],rotated_angles[1])
        #rospy.loginfo("servo angles" + str(servo_angles))
        sendCommand("F", servo_angles)

    elif nlostframes == 30: #not robust to failed recoveries
        past_errs.clear()
        sendCommand("T",[-1,-1,-1])

def sendCommand(resetstr, angles):
    angles_int = np.rint(angles)
    angles_set = np.clip(angles_int, 0, 65)
    #rospy.loginfo(angles_set)
    cmd = "<" + resetstr + "," + str(angles_set[0]) + "," +str(angles_set[1]) +"," + str(angles_set[2]) + ">"
    #rospy.loginfo(cmd.encode())
    arduino.write(cmd.encode())

#Takes in cropped and straightened image of platform and returns ball state
#dev note: after implementing quite a bit of framework to use a tracking algorithm
#rather than finding the ball from scratch each frame, I discovered that most algorithms
#are mysteriously not supported in opencv 4.5.5-dev. MIL, which I am using has been 
#reported to have issues recording failures, which I am experiencing as well
#The other options are DaSiamRPN and GOTURN, both of which require extra setup.
def track_ball(platform_image):
    global ball_on_platform
    global ball_loc
    global nlostframes
    global last_image
    
    ball_found = False
    motion_detected = False

    motion_detected, ball_loc = identify_motion(platform_image)

    if not motion_detected: #maybe the ball is stationary
        ball_found, ball_loc = identify_ball(platform_image)

    if (not ball_found) and (not motion_detected):
        ball_on_platform=False
        rospy.loginfo('no ball found for ' + str(nlostframes) + ' frames')
        nlostframes += 1
        last_image = None
        return False, None
    else:
        #rospy.loginfo('ball found' + str(nlostframes))
        nlostframes = 0
        (x,y,w,h)= [int(v) for v in ball_loc]
        #debug
        tracked=cv2.rectangle(platform_image, (x,y), (x+w, y+h), (0,255,0), 2)
        #cv2.imshow('box',tracked)
        #cv2.imwrite('processed_image.jpg',tracked)
        cv2.waitKey(1)

        last_image = platform_image
        return True, np.array([x+w/2,y+h/2], np.single) 


def identify_ball(image):
    #may want to gaussian blur here
    #TODO:tune
    _,thresh_high = cv2.threshold(image, 235,255, cv2.THRESH_BINARY)
    _,thresh_low = cv2.threshold(image, 145,255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((5,5), np.uint8)
    try:
        high_dilated = cv2.dilate(thresh_high,kernel, iterations = 3) #TODO: tune
        low_dilated = cv2.dilate(thresh_low,kernel, iterations = 3) #TODO: tune
        ball_mask=cv2.bitwise_and(high_dilated,low_dilated)

        #cv2.imshow('brightest', high_dilated)
        #cv2.imshow('undilated', thresh_high)
        #cv2.imshow('darkest', thresh_low)
    except:
        rospy.loginfo('no sufficiently bright/dark spots found')
        ball_mask=0


    #cv2.imshow('ball_mask', ball_mask)
    #cv2.waitKey(1)

    #print(np.sum(thresh_eroded))
    if (np.sum(ball_mask) > 2000): #TODO:Tune
    #    contours=cv2.findContours(thresh_eroded,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return(True, cv2.boundingRect(ball_mask))
    else:
        return False, [-1,-1,-1,-1]

def identify_motion(image):
    global last_image
    global maze_mask_eroded
    
    #rospy.loginfo('using motion')
    image = cv2.blur(image, (3,3))

    if last_image is None:
        last_image = image
        return(False, [-1,-1,-1,-1])

    delta = cv2.absdiff(image,last_image)
    _, delta_thresh = cv2.threshold(delta, 50, 255, cv2.THRESH_BINARY)
    delta_thresh_masked = np.bitwise_and(delta_thresh, maze_mask_eroded)
    delta_thresh_masked_eroded = cv2.erode(delta_thresh_masked, np.ones((5,5), np.uint8))
    #cv2.imshow('difft',delta_thresh_masked_eroded)
    #cv2.waitKey(1)
    last_image = image
    if (np.sum(delta_thresh_masked_eroded) > 300): #TODO:tune
        return(True, cv2.boundingRect(delta_thresh_masked_eroded))
    else:
        return(False, [-1,-1,-1,-1])

def crop_platform(image):
    global maze_mask
    global prev_marker_centroids
    #Bounding regions of tape, as x,y,w,h, starting at the top left and going clockwise
    #found experimentally
    #TODO: tune to smaller bounding boxes

    marker_boxes=[
    [0,350,100,150],
    [350,0,150,200],
    [550,200,200,150],
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
    ptransform = cv2.getPerspectiveTransform(np.float32(marker_centroids),np.float32([[46,313],[517,43],[670,312],[203,582]]))
    straight = cv2.warpPerspective(image_bw, ptransform, (maze_mask.shape[1], maze_mask.shape[0]), flags = cv2.INTER_LINEAR)
    straight_clean = np.bitwise_and(straight,maze_mask)
    #cv2.imshow('straightened', straight_clean)
    
    return straight_clean

def crop(img, bb):
    return  img[bb[1]:bb[1]+bb[3],bb[0]:bb[0]+bb[2]]

def get_tape_centroid(image,bb):
    local_image = crop(image,bb)
    img_hsv=cv2.cvtColor(local_image, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(img_hsv,(133,41,215),(171,247,255))
    M = cv2.moments(thresh)
    #cv2.imshow('local_image',local_image)
    #cv2.imshow('thresh', thresh)
    #cv2.waitKey(1)
    if M["m00"]!=0: 
        cent_local =  [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])]
        return [bb[0]+cent_local[0],bb[1]+cent_local[1]]
    else:
        return None

def init_stewart():
    pbi = [[8.25, 0, 1.6], [-4.125, 7.145, 1.6], [-4.125, -7.145, 1.6]]
    ppi = [[8.805, 0, 0], [-4.402, 7.625, 0], [-4.402, -7.625, 0]]
    pa = 8.55
    ps = 8.55
    pbeta_i = [0, 120, 240]
    ph0 = 5
    
    # 100.53 mm from center
    # rest position with arm flat
    # platform anchors 35.346mm below platform proper.
    
    r0 = 13.818  # cm, radial displacement from base of servo joints
    arm = 8
    leg = 8
    servo_orientations = [0, 120, 240]  # even distribution of 3 angles wrt x
    h0 = leg  # platform height when all servos lay flat
    servo_0 = np.pi*45/180 # servo angle that has the arms level with the x-y plane
    
    base_anchors = [[r0, 0, 0],
                    [-1*r0*np.sin(np.pi/6), r0*np.cos(np.pi/6), 0],
                    [-1*r0*np.sin(np.pi/6), -1*r0*np.cos(np.pi/6), 0]]
    platform_anchors = [[(r0 + arm), 0, 0],
                        [-1*(r0 + arm)*np.sin(np.pi/6), (r0 + arm)*np.cos(np.pi/6), 0],
                        [-1*(r0 + arm)*np.sin(np.pi/6), -1*(r0+arm)*np.cos(np.pi/6), 0]]
    
    
    
    return Stewart.Stewart(base_anchors, platform_anchors, leg, arm, servo_orientations, h0,[40,35,42])


def listener():
    
    rospy.init_node('frame_processor', anonymous=True)
    
    rospy.Subscriber('picam_image', Image, img_callback)
    rospy.Subscriber('target_point', Point, target_callback)
    angle_pub=rospy.Publisher('target_platform_angle', Quaternion, queue_size=1)
    state_pub=rospy.Publisher('ball_on_platform', Bool, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
    theseus = init_stewart()
    arduino = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    global rotation_matrix
    theta = np.pi/3 #+np.pi #angle to align image coordinates with platform roll/pitch
    rotation_matrix=[[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]]
    
    global prev_marker_centroids #initialiation to rough locations
    prev_marker_centroids = [[50,400],[425,100],[650,325],[325,625]]

    global last_image
    last_image = None
    global target_point 
    target_point = np.array([300,100], np.single) #arbitrary default 
    global ball_on_platform
    ball_on_platform = False
    global nlostframes
    nlostframes = 0 
    global maze_mask
    maze_mask=cv2.imread('/home/pi/ros_catkin_ws/src/project_theseus/scripts/straightened_platform_mask.jpg' ,cv2.IMREAD_GRAYSCALE)
    global maze_mask_eroded
    maze_mask_eroded = cv2.erode(maze_mask,np.ones((5,5), np.uint8), iterations = 6) #TODO: tune
    global past_errs
    past_errs = deque(np.float32([0,0]),3)
    global target
    listener()
