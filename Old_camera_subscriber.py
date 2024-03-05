#! /usr/bin/env python3

from ctypes import pointer
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import math
import cv2
from math import *
from ros_open import _UTM_Class
from ros_open import Lane_Intersection_of_lines as Line_intersection
from ros_open import Marking_Line as Marking
from ros_open import find_parameter

output = int
init_Northing = float
init_Easting = float
initial_loop = 8
state = True

def first_ten_value(Northing, Easting):
  
  global initial_loop
  global init_Northing, init_Easting
  
  if (initial_loop != 0):
    init_Northing = Northing
    init_Easting = Easting
    initial_loop -= 1
  elif (initial_loop == 0):
    print("Ready to go!")
  print("init_Northing: ",init_Northing)
  print("init_Easting: ", init_Easting)
  
def destination (Northing, Easting):
   
  distinction_end1_latitude = 37
  distinction_end1_longitude = 20
  
  distinction_end2_latitude = 9
  distinction_end2_longitude = 6
  
  global init_Northing, init_Easting, output
  
  start_intersection1 = {
    "latitude" : init_Northing - distinction_end1_latitude,
    "longitude" : init_Easting - distinction_end1_longitude,
  }
  
  start_intersection2 = {
    "latitude" : start_intersection1["latitude"] - distinction_end2_latitude,
    "longitude" : start_intersection1["longitude"] - distinction_end2_longitude
    
  }
  
  if ((start_intersection2["latitude"]<Northing<(start_intersection1["latitude"])) and (start_intersection2["longitude"]<Easting<(start_intersection1["longitude"]))):
    output = 1
  else:
    output = 0
  
def callback_image(data):
  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")

  # Convert ROS Image message to OpenCV image
  frame = br.imgmsg_to_cv2(data)
  
  image_width = 540
  image_height = 640
  
  dst = cv2.resize(frame, (image_width,image_height))
  height = 500
  width = 270
  
  region_of_interest_vertices1 = [
                            (0,height),
                            (width-50, 240),
                            (width+50, 240),
                            (640, height)
  ]
  
  region_of_interest_vertices2 = [
                            (0,500),
                            (0,280),
                            (640,280),
                            (640,500)
  ]
  
  region_of_interest_vertices3 = [
                            (0,470),
                            (0,280),
                            (640,280),
                            (640,470)
  ]
  
  try:
    if (output==1):
        input_region_of_interest=region_of_interest_vertices2
        '''
        gaussian_para = 9
        
        canny_edges_min = 25
        canny_edges_max = 30
        
        input_threshold=75
        input_minLineLength=25
        input_maxLineGap=100
        '''
        gaussian_para = 13
        
        canny_edges_min = 50
        canny_edges_max = 50 
        
        input_threshold=75
        input_minLineLength=25
        input_maxLineGap=100
        
    elif(output==2):
        input_region_of_interest=region_of_interest_vertices3
        
        gaussian_para = 13
        
        canny_edges_min = 50
        canny_edges_max = 50
        
        input_threshold=70
        input_minLineLength=50
        input_maxLineGap=100
        
    else:
        input_region_of_interest=region_of_interest_vertices1
        
        gaussian_para = 13
        
        canny_edges_min = 35
        canny_edges_max = 45
        
        input_threshold=90
        input_minLineLength=45
        input_maxLineGap=130
        
  except NameError:
    output == 0
    input_region_of_interest=region_of_interest_vertices1
 
  grayImage = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
  grayImage = cv2.GaussianBlur(grayImage,(gaussian_para,gaussian_para),0)
  edges = cv2.Canny(grayImage, canny_edges_min, canny_edges_max)
  cropped_image = Line_intersection.region_of_interest(edges, np.array([input_region_of_interest], np.int32))
  lines = cv2.HoughLinesP(cropped_image, rho=1, theta=np.pi/180, threshold = input_threshold ,minLineLength = input_minLineLength, maxLineGap = input_maxLineGap)

  cv2.imshow("Cropped Image", cropped_image)
  Marking.marking_Lanes(dst, lines, output)
  
  cv2.waitKey(250)

def callback_gps(gps) :

  print("GPS Longitude and Latitude Value \n")
  # Latitude (degrees). Positive is north of equator; negative is south.
  # Longitude (degrees). Positive is east of prime meridian, negative west.
  
  latitude = round(gps.latitude, 5)
  longitude = round(gps.longitude, 5)
  
  print("Longitude: "+ str(longitude)) #This will print the whole GPS Message
  print("Latitude: "+ str(latitude))
  
  Northing, Easting, Zone = _UTM_Class.LLtoUTM(latitude, longitude)
  
  first_ten_value(Northing, Easting)
  
  Northing = round(Northing, 1)
  Easting = round(Easting, 1)
  
  destination(Northing, Easting)
  
  print("Output: ", output)
  print("Latitude: ", Northing)
  print("longitude: ", Easting)
  
def callback_gps_value(data):
  global output
  output = int(data.data)
  print(data.data)
  
def receive_message():
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_sub_py', anonymous=True)

  # Node is subscribing to the /camera/image topic
  #rospy.Subscriber('gps_location', Int32, callback_gps_value)
  
  rospy.Subscriber('/gps/fix', NavSatFix, callback_gps)
  rospy.Subscriber('/camera/image_color', Image, callback_image)
  
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

  # Close down the video stream when done
  cv2.destroyAllWindows()

if __name__ == '__main__':
  receive_message()
