#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


bridge = CvBridge()
cv_image = np.empty(shape=[0])
#cv_image = np.empty((640,480))

def callback(data):
  global cv_image
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the /camera/image topic
  rospy.Subscriber('/camera/image_color', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
