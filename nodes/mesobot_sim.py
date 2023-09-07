#!/usr/bin/env python3

import rospy
import tf.transformations

import mesobot_project11.dynamics

from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPointStamped
from sonardyne_msgs.msg import SMS
from std_msgs.msg import String

import math


class USBL:
  def __init__(self):
    self.tracking_period = rospy.Duration(10.0) # time between tracking updates
    self.max_range = 3000 # distance in meters

    self.frame_id = rospy.get_param('~usbl_frame_id', 'base_link')
    self.address =  rospy.get_param('~usbl_address', 2509)

    self.last_tracking_time = None

    self.position_pub = rospy.Publisher('position', GeoPointStamped, queue_size=1)

    self.sms_sub = rospy.Subscriber('send_sms', SMS, self.usblSMSCallback)
    self.raw_sub = rospy.Subscriber('send_raw', String, self.usblRawCallback)

  def iterate(self, event):
    if self.last_tracking_time is None or event.current_real - self.last_tracking_time >= self.tracking_period:
      gps = GeoPointStamped()
      gps.header.stamp = event.current_real
      gps.position.latitude = mesobot.latitude
      gps.position.longitude = mesobot.longitude
      gps.position.altitude = -mesobot.depth
      self.position_pub.publish(gps)
      self.last_tracking_time = event.current_real

  def usblRawCallback(self, msg):
    pass

  def usblSMSCallback(self, msg):
    pass


mesobot = mesobot_project11.dynamics.MesobotDynamics()

rospy.init_node("mesobot_sim")


lat = rospy.get_param('~start_latitude', 43.0)
lon = rospy.get_param('~start_longitude', -70.65)
depth = rospy.get_param('~start_depth', 50.0)
heading = rospy.get_param("~start_heading", 6.0)

mesobot.reset(lat, lon, depth, heading, rospy.get_time())

pose_pub = rospy.Publisher('~debug/position', GeoPoseStamped, queue_size=1)

def iterate(event):
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
