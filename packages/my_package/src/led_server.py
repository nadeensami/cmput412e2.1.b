#!/usr/bin/env python3
import rospy
import sys
# from my_package.srv import ColorServiceResponse, ColorService
# from std_msgs.msg import Bool, String
from std_srvs.srv import SetBoolResponse
from duckietown_msgs.srv import SetFSMState
from duckietown.dtros import DTROS, NodeType

NAME = 'led_server'

class LEDServerNode(DTROS):
  def __init__(self, node_name):
    super(LEDServerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
    self.service = rospy.Service(NAME, SetFSMState, self.change_color)

  def change_color(self, request):
    print("got request: ", request)
    response = SetBoolResponse()
    response.success = True
    response.message = 'Successfully set color'
    return response

if __name__ == '__main__':
  # Create the LEDServerNode object
  led_emitter_node = LEDServerNode(node_name="led_server_node")
  # Keep it spinning to keep the node alive
  rospy.spin()