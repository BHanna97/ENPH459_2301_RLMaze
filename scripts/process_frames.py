#!/usr/bin/env python

import cv2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

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
    get_command()

#TODO:take in an image as a numpy array. If the ball cannot be found on the platform,
#set ball_on_platform to 0, and send a null(?) value to target_platform_angle.
#Otherwise, publish the desired angle of the plaform to target_platform_angle
def get_command():
	return False

def listener():

    rospy.init_node('frame_processor', anonymous=True)

    rospy.Subscriber('picam_image', Image, callback)
    
    angle_pub=rospy.Publisher('target_platform_angle', Quaternion, queue_size=1)
    state_pub=rospy.Publisher('ball_on_platform', Bool, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    bridge = CvBridge()
    listener()
