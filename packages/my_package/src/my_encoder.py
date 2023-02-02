#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):
  def __init__(self, node_name):
    """
    Wheel Encoder Node
    This implements basic functionality with the wheel encoders.
    """

    # Initialize the DTROS parent class
    super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
    self.veh_name = rospy.get_namespace().strip("/")

    # Get static parameters
    self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

    # Subscribing to the wheel encoders
    # self.sub_encoder_ticks_left = rospy.Subscriber(f"/{self.veh_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_callback)
    # self.sub_encoder_ticks_right = rospy.Subscriber(f"/{self.veh_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_callback)
    self.sub_executed_commands = rospy.Subscriber(f"/{self.veh_name}/wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.command_callback)

    # /csc22905/left_wheel_encoder_node/tick /csc22905/right_wheel_encoder_node/tick /csc22905/wheels_driver_node/wheels_cmd_executed
    # Publishers
    # self.pub_integrated_distance_left = rospy.Publisher(...)
    # self.pub_integrated_distance_right = rospy.Publisher(...)

    self.log("Initialized")
  
  def left_callback(self, data):
    rospy.loginfo("Left data: %s", data.data)
  
  def right_callback(self, data):
    rospy.loginfo("Right data: %s", data.data)
  
  def command_callback(self, data):
    rospy.loginfo("Left velocity data: %s", data.vel_left)
    rospy.loginfo("Right velocity data: %s", data.vel_right)

  def cb_encoder_data(self, wheel, msg):
    """
    Update encoder distance information from ticks.
    """

  def cb_executed_commands(self, msg):
    """
    Use the executed commands to determine the direction of travel of each wheel.
    """

if __name__ == '__main__':
  node = OdometryNode(node_name='my_encoder_node')
  # Keep it spinning to keep the node alive
  rospy.spin()
  rospy.loginfo("wheel_encoder_node is up and running...")