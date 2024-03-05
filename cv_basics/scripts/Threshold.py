#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from FLIR Camera.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from std_msgs.msg import Float64
 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  
  dst = cv2.resize(current_frame, (540,640))
  
  grayImage = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
  grayImage = cv2.GaussianBlur(grayImage,(9,9),0)
  ret,grayImage7 = cv2.threshold(grayImage, 85, 255, cv2.THRESH_BINARY)
  edges7 = cv2.Canny(grayImage7, 50, 50)
  #edges31 = cv2.Canny(grayImage31, 180, 180)
   
  # Display image
  cv2.imshow("camera", dst)
  cv2.imshow("edges", edges7)
  cv2.imshow("Threshold Image", grayImage7)
  
  cv2.waitKey(1)

def callback_IMU (data):
  
  print(data)
  
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
