#!/usr/bin/env python3
import numpy as np
import os, math
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from duckietown_msgs.srv import SetFSMState
from std_msgs.msg import Header, Float32

NAME = 'led_server'

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
    self._L = 0.05

    # Nadeen's Robot
    # self.right_velocity = 0.4297693967819214
    # self.left_velocity = 0.5252736806869507

    rospy.on_shutdown(self.clean_shutdown)

    # Celina's Robot
    self.right_velocity = 1.5 * 0.4779990613460541
    self.left_velocity = 1.5 * 0.477044016122818

    # Initialize the executed commands message
    self.msg_wheels_cmd = WheelsCmdStamped()
    self.msg_wheels_cmd.header = Header()

    # Subscribers
    self.sub_encoder_ticks_left = rospy.Subscriber(f"/{self.veh_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_callback)
    self.sub_encoder_ticks_right = rospy.Subscriber(f"/{self.veh_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_callback)
    # self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.command_callback)

    # Publishers
    self.pub_integrated_distance = rospy.Publisher('distance_travelled', Float32, queue_size=10)
    self.pub_wheel_command = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    self.initial_ticks_left = -1
    self.current_ticks_left = 0
    self.initial_ticks_right = -1
    self.current_ticks_right = 0

    self.log("Initialized")


    rospy.wait_for_service(NAME)
    self.change_color = rospy.ServiceProxy(NAME, SetFSMState)
    self.change_color("white")
  
  def left_callback(self, msg):
    # rospy.loginfo("Left data: %s", msg.data)
    self.current_ticks_left = msg.data
    if self.initial_ticks_left < 0: self.initial_ticks_left = msg.data # set initial ticks if uninitialized

  def right_callback(self, msg):
    # rospy.loginfo("Right data: %s", msg.data)
    self.current_ticks_right = msg.data
    if self.initial_ticks_right < 0: self.initial_ticks_right = msg.data # set initial ticks if uninitialized

  def command_callback(self, data):
    rospy.loginfo("Left velocity data: %s", data.vel_left)
    rospy.loginfo("Right velocity data: %s", data.vel_right)

  def runDistancePublisher(self):
    # publish message every 2 seconds
    rate = rospy.Rate(0.5) # 0.5Hz
    while not rospy.is_shutdown():
      self.pub_integrated_distance.publish(self.distanceTravelled())
      rate.sleep()

  def clean_shutdown(self):
    self.publishCommand(0.0, 0.0)

  def run(self):
    rate = rospy.Rate(10) # 10 times a second
    substate = ["change-color", "right-turn", "forward", "left-turn", "forward", "left-turn", "forward", "left-turn", "forward", "left-turn", "left-turn","left-turn"]
    i = 0
    state = 1
    rate_5s = rospy.Rate(0.2) # 5 second rate
    while not rospy.is_shutdown():
      if state == 1:
        self.change_color("cyan")
        rate_5s.sleep()
        state += 1
      elif state == 2:
        if i >= len(substate):
          self.publishCommand(0.0, 0.0)
          self.resetInitialTicks()
          self.change_color("cyan")
          rate_5s.sleep()
          state += 1
        elif substate[i] == "change-color":
          self.change_color("purple")
          i += 1
        elif substate[i] == "forward":
          if self.distanceTravelled() < 1.1:
            self.publishCommand(self.left_velocity, self.right_velocity)
          else:
            self.publishCommand(0.0, 0.0)
            self.resetInitialTicks()
            i += 1
        elif substate[i] == "right-turn":
          if self.angleTurned() < 1.535:
            self.publishCommand(self.left_velocity, -self.right_velocity)
          else:
            self.publishCommand(0.0, 0.0)
            self.resetInitialTicks()
            i += 1
        elif substate[i] == "left-turn":
          if self.angleTurned() < 1.545:
            self.publishCommand(-self.left_velocity, self.right_velocity)
          else:
            self.publishCommand(0.0, 0.0)
            self.resetInitialTicks()
            i += 1
      rate.sleep()
    self.publishCommand(0.0, 0.0)

  def publishCommand(self, left_vel, right_vel):
    self.msg_wheels_cmd.header.stamp = rospy.Time.now()
    self.msg_wheels_cmd.vel_left = left_vel
    self.msg_wheels_cmd.vel_right = right_vel
    self.pub_wheel_command.publish(self.msg_wheels_cmd)

  def distanceTravelledRight(self):
    return 2 * math.pi * self._radius * (self.current_ticks_right - self.initial_ticks_right) / self._total_ticks
  
  def resetInitialRightTicks(self):
    self.initial_ticks_right = self.current_ticks_right
  
  def distanceTravelledLeft(self):
    return 2 * math.pi * self._radius * (self.current_ticks_left - self.initial_ticks_left) / self._total_ticks
  
  def resetInitialLeftTicks(self):
    self.initial_ticks_left = self.current_ticks_left

  def resetInitialTicks(self):
    self.resetInitialLeftTicks()
    self.resetInitialRightTicks()
  
  def distanceTravelled(self):
    return (self.distanceTravelledRight() + self.distanceTravelledLeft()) / 2
  
  def angleTurned(self):
    return abs(self.distanceTravelledRight() - self.distanceTravelledLeft()) / (2 * self._L)

if __name__ == '__main__':
  node = OdometryNode(node_name='my_encoder_node')
  # Run the publisher
  # node.runDistancePublisher()
  # Keep it spinning to keep the node alive
  node.run()
  rospy.spin()
  rospy.loginfo("wheel_encoder_node is up and running...")