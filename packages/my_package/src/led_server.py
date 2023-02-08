#!/usr/bin/env python
import rospy
import sys
from my_package.srv import *

NAME = 'led_server'

# request.color
def change_color(request):
  print("got request: ", request.color)
  return ColorServiceResponse(True)

def change_color_service():
  rospy.init_node(NAME)
  s = rospy.Service('change_color', ColorService, change_color)

  rospy.spin()

if __name__ == '__main__':
#   print("List of modules: ", dir())
#   print("system path: ", sys.path)
  change_color_service()