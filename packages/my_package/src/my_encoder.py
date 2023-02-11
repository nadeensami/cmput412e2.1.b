#!/usr/bin/env python3
import math, time, rospy, rosbag
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from duckietown_msgs.srv import SetFSMState
from std_msgs.msg import Header, Float32, String
import message_filters

NAME = 'led_server'

class OdometryNode(DTROS):
  def __init__(self, node_name):
    """
    Wheel Encoder Node
    This implements basic functionality with the wheel encoders.
    """
    # Initialize start time
    self._start_time = time.time()

    # Initialize the DTROS parent class
    super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
    self.veh_name = rospy.get_namespace().strip("/")

    self._state_color = {1: "cyan", 2: "purple", 3: "yellow", 4: "green"}

    # Static parameters
    self._radius = 0.0318
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
    self.sub_encoder_ticks_left = message_filters.Subscriber(f"/{self.veh_name}/left_wheel_encoder_node/tick", WheelEncoderStamped)
    self.sub_encoder_ticks_right = message_filters.Subscriber(f"/{self.veh_name}/right_wheel_encoder_node/tick", WheelEncoderStamped)

    self.sub_encoder_ticks_left.registerCallback(self.left_callback)
    self.sub_encoder_ticks_right.registerCallback(self.right_callback)

    # Publishers
    self.pub_integrated_distance = rospy.Publisher('distance_travelled', Float32, queue_size=10)
    self.pub_wheel_command = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    # Setup the time synchronizer
    self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
      [self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.5
    )

    self.ts_encoders.registerCallback(self.cb_ts_encoders)

    self.initial_ticks_left = -1
    self.current_ticks_left = 0
    self.initial_ticks_right = -1
    self.current_ticks_right = 0

    self.log("Initialized")

    rospy.wait_for_service(NAME)
    self.change_color = rospy.ServiceProxy(NAME, SetFSMState)
    self.change_color("white")

    self.bag = rosbag.Bag('/data/bags/odometry.bag', 'w')
    self.bag_string = String()
    self.position = {'x': 0.32, 'y': 0.32, 'theta': math.pi/2}

    '''
      Constants for self.ticks_per_meter and self.encoder_stale_dt taken from "jihoonog/CMPUT-503-Exercise-2"
      link: https://github.com/jihoonog/CMPUT-503-Exercise-2/blob/561e08a558a95b95e2c01dd21c2394f2f9b4fb34/packages/exercise-2/src/motor_control_node.py
      AUTHORS: Jihoon Og, Qianxi Li
    '''
    self.ticks_per_meter = 656
    self.encoder_stale_dt = 1.0

    self.left_encoder_last = None
    self.right_encoder_last = None
    self.encoders_timestamp_last = None
    self.encoders_timestamp_last_local = None

    self.timestamp = None
    # initial coordinates according to exercise instructions are
    # (0.32, 0.32), with theta = pi/2
    self.x = 0.32
    self.y = 0.32
    self.yaw = math.pi/2
    self.tv = 0.0
    self.rv = 0.0

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

  def writeOdometryInformation(self, x, y, theta):
    self.bag_string.data = f"({x}, {y}, {theta})"
    print(f"({x}, {y}, {theta})")
    self.bag.write('odometry', self.bag_string)

  def clean_shutdown(self):
    self.change_color("off")
    self.publishCommand(0.0, 0.0)
    self.bag.close()

  def run(self):
    rate = rospy.Rate(10) # 10 times a second
    tasks = [
      # Step 2
      "state-1",
      # Step 3
      "state-2", "right-turn", "forward", "left-turn", "forward",
      # Step 4
      "state-1",
      # Step 5
      "state-3", "left-turn", "forward", "left-turn", "forward", "left-turn", "left-turn", "left-turn",
      # Step 6
      "state-1",
      # Step 7
      "state-4", "circular-turn"
    ]
    i = 0
    while not rospy.is_shutdown() and i < len(tasks):
      if tasks[i] == "state-1":
        self.change_color(self._state_color[1])
        time.sleep(5)        
        i += 1

      elif tasks[i] == "state-2":
        self.change_color(self._state_color[2])
        i += 1

      elif tasks[i] == "state-3":
        self.change_color(self._state_color[3])
        i += 1

      elif tasks[i] == "state-4":
        self.change_color(self._state_color[4])
        i += 1

      elif tasks[i] == "forward":
        if self.distanceTravelled() < 1.1:
          self.publishCommand(self.left_velocity, self.right_velocity)
        else:
          self.publishCommand(0.0, 0.0)
          self.resetInitialTicks()
          i += 1

      elif tasks[i] == "right-turn":
        if self.angleTurned() > -1.51:  # a little less than pi/2 (90 degree turn)
          self.publishCommand(self.left_velocity, -self.right_velocity)
        else:
          self.publishCommand(0.0, 0.0)
          self.resetInitialTicks()
          i += 1
      
      elif tasks[i] == "left-turn":
        if self.angleTurned() < 1.53:  # a little less than pi/2 (90 degree turn)
          self.publishCommand(-self.left_velocity, self.right_velocity)
        else:
          self.publishCommand(0.0, 0.0)
          self.resetInitialTicks()
          i += 1

      elif tasks[i] == "circular-turn":
        if self.angleTurned() > -5.8:  # a little less than 2*pi
          self.publishCommand(self.left_velocity, self.right_velocity  * 0.5)
        else:
          self.publishCommand(0.0, 0.0)
          self.resetInitialTicks()
          i += 1 
      
      rate.sleep()

    self.publishCommand(0.0, 0.0)

    end_time = time.time()
    # 8
    print("DONE PROGRAM. Total execution time: ", end_time - self._start_time)

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
    return (self.distanceTravelledRight() - self.distanceTravelledLeft()) / (2 * self._L)

  '''
    Code for functions cb_ts_encoders(), cb_timer(), angle_clamp() taken from
    "duckietown/dt-core", file "deadreckoning_node.py"
    Link: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py
    AUTHORS: Matthew Walter, Andrea F Daniele, Andrea Censi
  '''
  def cb_ts_encoders(self, left_encoder, right_encoder):
    timestamp_now = rospy.get_time()

    # Use the average of the two encoder times as the timestamp
    left_encoder_timestamp = left_encoder.header.stamp.to_sec()
    right_encoder_timestamp = right_encoder.header.stamp.to_sec()
    timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

    if not self.left_encoder_last:
      self.left_encoder_last = left_encoder
      self.right_encoder_last = right_encoder
      self.encoders_timestamp_last = timestamp
      self.encoders_timestamp_last_local = timestamp_now
      return

    # Skip this message if the time synchronizer gave us an older message
    dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
    dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
    if dtl.to_sec() < 0 or dtr.to_sec() < 0:
      rospy.logwarn("Ignoring stale encoder message")
      return

    left_dticks = left_encoder.data - self.left_encoder_last.data
    right_dticks = right_encoder.data - self.right_encoder_last.data

    left_distance = left_dticks * 1.0 / self.ticks_per_meter
    right_distance = right_dticks * 1.0 / self.ticks_per_meter

    # Displacement in body-relative x-direction
    distance = (left_distance + right_distance) / 2

    # Change in heading
    dyaw = (right_distance - left_distance) / (2 * self._L)

    dt = timestamp - self.encoders_timestamp_last

    if dt < 1e-6:
      rospy.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
      return

    self.tv = distance / dt
    self.rv = dyaw / dt

    self.yaw = self.angle_clamp(self.yaw + dyaw)
    self.x = self.x + distance * math.cos(self.yaw)
    self.y = self.y + distance * math.sin(self.yaw)
    self.timestamp = timestamp

    self.left_encoder_last = left_encoder
    self.right_encoder_last = right_encoder
    self.encoders_timestamp_last = timestamp
    self.encoders_timestamp_last_local = timestamp_now

    # Write odometry to bag
    self.writeOdometryInformation(self.x, self.y, self.yaw)    

  def cb_timer(self, _):
    if self.encoders_timestamp_last:
      dt = rospy.get_time() - self.encoders_timestamp_last_local
      if abs(dt) > self.encoder_stale_dt:
        self.rv = 0.0
        self.tv = 0.0
    else:
      self.rv = 0.0
      self.tv = 0.0

  @staticmethod
  def angle_clamp(theta):
    if theta > 2 * math.pi:
      return theta - 2 * math.pi
    elif theta < -2 * math.pi:
      return theta + 2 * math.pi
    else:
      return theta

if __name__ == '__main__':
  node = OdometryNode(node_name='my_encoder_node')
  # Run the node
  node.run()
  # Keep it spinning to keep the node alive
  rospy.spin()
  rospy.loginfo("wheel_encoder_node is up and running...")