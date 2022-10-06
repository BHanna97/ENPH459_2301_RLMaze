#! /usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
 
# To open matplotlib in interactive mode
#%matplotlib qt5
 
 
# Load the image
img = cv2.imread('/home/pi/ros_catkin_ws/src/project_theseus/scripts/straightened_platform_mask.jpg') 
 
# Create a copy of the image
img_copy = np.copy(img)
 
# Convert to RGB so as to display via matplotlib
# Using Matplotlib we can easily find the coordinates
# of the 4 points that is essential for finding the 
# transformation matrix
img_copy = cv2.cvtColor(img_copy,cv2.COLOR_BGR2RGB)
 
plt.imshow(img_copy)

plt.show()
