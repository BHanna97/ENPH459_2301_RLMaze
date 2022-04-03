#!/usr/bin/env python

#Modified from https://picamera.readthedocs.io/en/release-1.13/recipes2.html?highlight=capture_continuous#rapid-capture-and-processing

import io
import time
import threading
import picamera

import PIL.Image
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor(threading.Thread):
    def __init__(self, owner):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()
        
        self.pub = rospy.Publisher('picam_image', Image, queue_size=1)
        rospy.init_node('picam_publisher', anonymous=True)
        self.bridge = CvBridge()

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    #There may be more conversions than necessary here, but I couldn't find
                    #any way to do this more directly
                    # 'L' for grayscale 'rbg' (?) for color
                    self.pil_image = PIL.Image.open(self.stream).convert('RGB')
                    self.ocv_image = np.array(self.pil_image)
                    self.ocv_image = self.ocv_image[:,:,::-1].copy()
                    # 'mono8' for grayscale 'bgr8' for color
                    self.image_message = self.bridge.cv2_to_imgmsg(self.ocv_image, encoding='bgr8')
                    self.pub.publish(self.image_message)


                        
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)
                    
                    if rospy.is_shutdown():
                        print("ending..")
                        self.owner.done=True
                        
class ProcessOutput(object):
    def __init__(self):
        self.done = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self) for i in range(8)]
        self.processor = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    rospy.loginfo("No processor available for frame")
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()

with picamera.PiCamera(resolution=(720,720), framerate = 10) as camera:
    #camera.color_effects = (128,128) #uncomment for black and white
    #camera.start_preview()
    #time.sleep(2)
    output = ProcessOutput()
    camera.start_recording(output, format='mjpeg')
    while not output.done:
        camera.wait_recording(1)
    output.flush()
    camera.stop_recording()
