#!/usr/bin/env python3
import numpy as np
import os, math
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
# from wheels_driver.dagu_wheels_driver import DaguWheelsDriver

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
    self._total_ticks = 135

    # Initialize the executed commands message
    self.msg_wheels_cmd = WheelsCmdStamped()
    self.msg_wheels_cmd.header = Header()

    # Subscribing to the wheel encoders
    self.sub_encoder_ticks_left = rospy.Subscriber(f"/{self.veh_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_callback)
    self.sub_encoder_ticks_right = rospy.Subscriber(f"/{self.veh_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_callback)
    # self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.command_callback)

    # /csc22905/left_wheel_encoder_node/tick /csc22905/right_wheel_encoder_node/tick /csc22905/wheels_driver_node/wheels_cmd_executed
    # Publishers
    # self.pub_integrated_distance_left = rospy.Publisher('distance_travelled_left', Float32, queue_size=10)
    # self.pub_integrated_distance_right = rospy.Publisher('distance_travelled_left', Float32, queue_size=10)
    self.pub_wheel_command = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    self.initialized_left = False
    self.initial_ticks_left = 0
    self.initialized_right = False
    self.initial_ticks_right = 0
    self.distance_travelled_left = 0
    self.distance_travelled_right = 0

    self.right_velocity = 0.4297693967819214
    self.left_velocity = 0.5252736806869507

    self.log("Initialized")
  
  def left_callback(self, data):
    # rospy.loginfo("Left data: %s", data.data)
    if not self.initialized_left:
      self.initialized_left = True
      self.initial_ticks_left = data.data
    else:
      self.distance_travelled_left = 2 * math.pi * self._radius * (data.data - self.initial_ticks_left) / self._total_ticks
  
  def right_callback(self, data):
    # rospy.loginfo("Right data: %s", data.data)
    if not self.initialized_right:
      self.initialized_right = True
      self.initial_ticks_right = data.data
    else:
      self.distance_travelled_right = 2 * math.pi * self._radius * (data.data - self.initial_ticks_right) / self._total_ticks
  
  def command_callback(self, data):
    rospy.loginfo("Header sequence: %s", data.header.seq)
    rospy.loginfo("Header stamp: %s", data.header.stamp)
    rospy.loginfo("Header frame id: %s", data.header.frame_id)
    rospy.loginfo("Header sequence: %s", data.header.seq)
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

  def runDistancePublisher(self):
    # publish message every 2 seconds
    rate = rospy.Rate(0.5) # 0.5Hz
    while not rospy.is_shutdown():
      rospy.loginfo("Left distance: %s", self.distance_travelled_left)
      self.pub_integrated_distance_left.publish(self.distance_travelled_left)

      rospy.loginfo("Right distance: %s", self.distance_travelled_right)
      self.pub_integrated_distance_right.publish(self.distance_travelled_right)
      rate.sleep()

  def run(self):
    # publish message every 1 second
    rate = rospy.Rate(1) # 1
    # self.driver.set_wheels_speed(left=self.left_velocity, right=self.right_velocity)
    while not rospy.is_shutdown():
      if max(self.distance_travelled_right, self.distance_travelled_left) >= 1.25:
        self.msg_wheels_cmd.header.stamp = rospy.Time.now()
        self.msg_wheels_cmd.vel_left = 0.0
        self.msg_wheels_cmd.vel_right = 0.0
        self.pub_wheel_command.publish(self.msg_wheels_cmd)
        # rospy.loginfo("Publishing command: %s", self.msg_wheels_cmd)
      else:
        self.msg_wheels_cmd.header.stamp = rospy.Time.now()
        self.msg_wheels_cmd.vel_left = self.left_velocity
        self.msg_wheels_cmd.vel_right = self.right_velocity
        self.pub_wheel_command.publish(self.msg_wheels_cmd)
      rospy.loginfo("Publishing command: %s", self.msg_wheels_cmd)
      rate.sleep()

if __name__ == '__main__':
  node = OdometryNode(node_name='my_encoder_node')
  # Run the publisher
  # node.runDistancePublisher()
  # Keep it spinning to keep the node alive
  node.run()
  rospy.spin()
  rospy.loginfo("wheel_encoder_node is up and running...")