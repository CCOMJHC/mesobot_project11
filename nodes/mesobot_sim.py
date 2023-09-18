#!/usr/bin/env python3

import rospy
import tf.transformations

import tf2_ros

import mesobot_project11.dynamics

from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPointStamped
from sonardyne_msgs.msg import SMS
from std_msgs.msg import String

import project11.wgs84

import math


class USBL:
  def __init__(self):
    self.tracking_period = rospy.Duration(10.0) # time between tracking updates
    self.max_range = 3000 # distance in meters

    self.frame_id = rospy.get_param('~usbl_frame_id', 'base_link')
    self.address = str(rospy.get_param('~usbl_address', 2509))

    self.last_tracking_time = None

    self.position_pub = rospy.Publisher('position', GeoPointStamped, queue_size=1)
    self.status_pub = rospy.Publisher('status', SMS, queue_size=1)

    self.sms_sub = rospy.Subscriber('send_sms', SMS, self.usblSMSCallback)
    self.raw_sub = rospy.Subscriber('send_raw', String, self.usblRawCallback)

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.distance = None

  def iterate(self, event):
    if self.last_tracking_time is None or event.current_real - self.last_tracking_time >= self.tracking_period:
      try:
        transform = self.tfBuffer.lookup_transform("earth", self.frame_id, rospy.Time())
        meso_ecef = project11.wgs84.toECEFfromDegrees(mesobot.latitude, mesobot.longitude, -mesobot.depth)
        
        dx = meso_ecef[0]-transform.transform.translation.x
        dy = meso_ecef[1]-transform.transform.translation.y
        dz = meso_ecef[2]-transform.transform.translation.z

        self.distance = math.sqrt(dx*dx+dy*dy+dz*dz)

        if self.distance < self.max_range:
          gps = GeoPointStamped()
          gps.header.stamp = event.current_real
          gps.position.latitude = mesobot.latitude
          gps.position.longitude = mesobot.longitude
          gps.position.altitude = -mesobot.depth
          self.position_pub.publish(gps)

          status_sms = SMS()
          status_sms.message = mesobot.status()
          self.status_pub.publish(status_sms)
        
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("transform error")
      self.last_tracking_time = event.current_real
      

  def usblRawCallback(self, msg):
    pass

  def usblSMSCallback(self, msg):
    if msg.address == self.address:
      if self.distance is not None and self.distance < self.max_range:
        self.executeCommand(msg.message)

  def executeCommand(self, command):
    mesobot.command(command, rospy.get_time())


mesobot = mesobot_project11.dynamics.MesobotDynamics()

rospy.init_node("mesobot_sim")


lat = rospy.get_param('~start_latitude', 43.03)
lon = rospy.get_param('~start_longitude', -70.67)
depth = rospy.get_param('~start_depth', 50.0)
heading = rospy.get_param("~start_heading", 6.0)

mesobot.reset(lat, lon, depth, heading, rospy.get_time())

pose_pub = rospy.Publisher('~debug/position', GeoPoseStamped, queue_size=1)

def iterate(event):
  mesobot.iterate(event.current_real.to_sec())
  gps = GeoPoseStamped()
  gps.header.frame_id = 'mesobot'
  gps.header.stamp = event.current_real
  gps.pose.position.latitude = mesobot.latitude
  gps.pose.position.longitude = mesobot.longitude
  gps.pose.position.altitude = -mesobot.depth

  yaw = math.radians(90.0)-mesobot.heading
  q = tf.transformations.quaternion_from_euler(yaw, 0, 0, 'rzyx')
  gps.pose.orientation.x = q[0]
  gps.pose.orientation.y = q[1]
  gps.pose.orientation.z = q[2]
  gps.pose.orientation.w = q[3]

  pose_pub.publish(gps)

  usbl.iterate(event)

usbl = USBL()

timer = rospy.Timer(rospy.Duration.from_sec(0.1), iterate)

rospy.spin()
