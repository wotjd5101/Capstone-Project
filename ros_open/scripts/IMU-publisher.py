#!/usr/bin/env python3
  
import rospy 
# the following line depends upon the 
# type of message you are trying to publish 
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
global orientation_z

def callback_imu(data):
    rospy.loginfo("orientation z: %f", data.orientation.z)
    
    orientation_z = data.orientation.z
    rospy.sleep(1)
  
if __name__ == '__main__': 
   rospy.init_node('IMU_Subscriber', anonymous=True)
   pub = rospy.Publisher('/orientaion_z', Float64, queue_size= 10)
   
   while not rospy.is_shutdown():
       
       rospy.Subscriber('/imu/data', Imu, callback_imu)