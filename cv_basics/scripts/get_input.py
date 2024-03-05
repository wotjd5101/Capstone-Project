#!/usr/bin/env python3

from rospy import init_node, is_shutdown, sleep,Publisher
import sys, select, termios, tty
from std_msgs.msg import Int32
global last_key
last_key = 0
 
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    
    int(last_key)
    

    if rlist:
        key = int(sys.stdin.read(1))
    else:
        key = last_key
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    return key

if __name__ == '__main__':
  init_node('input_test')
  settings = termios.tcgetattr(sys.stdin)
  pub = Publisher('gps_location', Int32, queue_size=10)

  while not is_shutdown():

      print ("gimme something, please?")
      input_key = getKey()
      int(input_key)
      last_key=input_key
      print("input_key: ", input_key)
      pub.publish(input_key)
      