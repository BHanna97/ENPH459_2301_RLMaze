#!/usr/bin/env python

import cv2

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

#TODO: store ball_on_platform as a locally acessible variable
def state_callback(data):
    return False

#TODO: Recieve the quaternion, convert to Euler angle (e.g. using tf.transformations) 
#and then to servo angles using reverse kinematics. Serialize this and send it over GPIO
def angle_callback(data):
    return False

def listener():
   
   rospy.init_node('motor_interface', anonymous=False)
   
   rospy.Subscriber('target_platform_angle', Quaternion, angle_callback)
   rospy.Subscriber('ball_on_platform', Bool, state_callback)
   
   rospy.spin()


if __name__ == '__main__':
   listener()
