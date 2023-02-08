#!/usr/bin/env python3
import rospy
# from my_package.srv import ColorServiceResponse, ColorService
# from std_msgs.msg import Bool, String
# from std_srvs.srv import SetBoolResponse
from duckietown_msgs.srv import SetFSMState
from duckietown.dtros import DTROS, NodeType

NAME = 'led_server'

class LEDClientNode(DTROS):
  def __init__(self, node_name):
    super(LEDClientNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

    rospy.wait_for_service(NAME)

    try:
      change_color = rospy.ServiceProxy(NAME, SetFSMState)
      resp = change_color("purple")
      print("got result: ", resp)
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)

if __name__ == '__main__':
  # Create the LEDClientNode object
  led_emitter_node = LEDClientNode(node_name="led_client_node")
  # Keep it spinning to keep the node alive
  rospy.spin()