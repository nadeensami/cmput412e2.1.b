#!/usr/bin/env python
import rospy
import sys
from my_package.srv import *

NAME = 'led_server'

def change_color_client():
  rospy.wait_for_service(NAME)
  try:
    change_color = rospy.ServiceProxy(NAME, ColorService)
    resp = change_color("purple")
    print("got result: ", resp.result)
    return resp.result    
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

if __name__ == '__main__':
  print(sys.path)
  change_color_client()