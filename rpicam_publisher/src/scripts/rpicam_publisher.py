#!/usr/bin/env python3
import roslib
import sys
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(1280, 960))
image_pub = rospy.Publisher("raspicam_custom", Image)
bridge = CvBridge()
rospy.init_node('raspicam_custom', anonymous=True)

time.sleep(0.1)

def main():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        # cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        try:
            image_pub.pubslish(bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    main()
    

