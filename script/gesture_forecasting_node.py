#!/usr/bin/env python

import os
import sys
import rospy
import cv2

sys.path.insert(0, '/home/leejang/lib/ssd_caffe/caffe/python')
import caffe

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class gesture_forecasting:

    def __init__(self):
     self.bridge = CvBridge()
     self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback) 
     self.gesture_pub = rospy.Publisher("forecasting/gesture", String, queue_size=10)

    def img_callback(self,data):
      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError, e:
        print e

      # write image

      #print('img_calback')
      self.do_gesture_forecasting();

    def do_gesture_forecasting(self):
      #print('start gesture forecasting!')
      #string = "come"
      #string = "go"
      #string = "stop"
      #string = "selfie"
      string = "go"
      self.gesture_pub.publish(string)

def main(args):
    print 'initialize gesture forecasting node (python)'

    gesture_node = gesture_forecasting()
    rospy.init_node('gesture_forecasting', anonymous=True)
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
