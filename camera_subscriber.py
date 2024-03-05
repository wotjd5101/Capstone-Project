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
from typing import Dict

output = int
init_Northing = float
init_Easting = float
initial_loop = 10
state = True
line_parameter = Dict

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
  
  print("Latitude Range: ",start_intersection2["latitude"]," <latitude< ",start_intersection1["latitude"])
  print("Longitude Range: ",start_intersection2["longitude"]," <longitude< ",start_intersection1["longitude"])
  
  if ((start_intersection2["latitude"]<Northing<(start_intersection1["latitude"])) and (start_intersection2["longitude"]<Easting<(start_intersection1["longitude"]))):
    output = 1
  else:
    output = 0
  
def callback_image(data):
  
  image_width = 540
  image_height = 640
  height = 500
  width = 270
  global line_parameter, state
  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")

  # Convert ROS Image message to OpenCV image
  frame = br.imgmsg_to_cv2(data)
  dst = cv2.resize(frame, (image_width,image_height))
  
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

    elif(output==2):
        input_region_of_interest=region_of_interest_vertices3
        
    else:
        input_region_of_interest=region_of_interest_vertices1
        
  except NameError:
    output == 0
    input_region_of_interest=region_of_interest_vertices1
  
  if (state):
    line_parameter = find_parameter.find_line(dst,output)
    
  gaussian_para = line_parameter['gaussian_para']
  canny_edges_min = line_parameter['canny_edges_min']
  canny_edges_max = line_parameter['canny_edges_max']
  input_threshold = line_parameter['input_threshold']
  input_minLineLength = line_parameter['input_minLineLength']
  input_maxLineGap = line_parameter['input_maxLineGap']
  state = False
  
  print("canny edge min 0: ", line_parameter['canny_edges_min'])
  print("canny edge max 0: ", line_parameter['canny_edges_max'])
  
  grayImage = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
  grayImage = cv2.GaussianBlur(grayImage,(gaussian_para,gaussian_para),0)
  edges = cv2.Canny(grayImage, canny_edges_min, canny_edges_max)
  cropped_image = Line_intersection.region_of_interest(edges, np.array([input_region_of_interest], np.int32))
  lines = cv2.HoughLinesP(cropped_image, rho=1, theta=np.pi/180, threshold = input_threshold ,minLineLength = input_minLineLength, maxLineGap = input_maxLineGap)
  print("Line after HoughLineP: ", lines)
  
  check_lines_status = np.any(lines)
  print("check lines status: ", check_lines_status)
  if check_lines_status == None:
    state = True
  else:
    Marking.marking_Lanes(dst, lines, output)
  

  lines = []
  
  cv2.imshow("Cropped Image", cropped_image)
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
  rospy.Subscriber('gps_location', Int32, callback_gps_value)
  
  #rospy.Subscriber('/gps/fix', NavSatFix, callback_gps)
  rospy.Subscriber('/camera/image_color', Image, callback_image)
  
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  cv2.destroyAllWindows()
  # Close down the video stream when done

if __name__ == '__main__':
  receive_message()
