#!/usr/bin/env python3

import rospy

from sonardyne_msgs.msg import SMS
from geographic_msgs.msg import GeoPointStamped
from geographic_msgs.msg import GeoPoseStamped
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue as HBKeyValue

import math

import project11
from tf.transformations import quaternion_about_axis

last_state = None

def smsCallback( msg):
  global last_state
  print (msg)

  decoder = {'H:':'Heading',
             'D:':'Depth',
             'B:':'Battery',
             'R:':'Radiometer',
             'S:':'Flows',
             'C:':'Command',
             'M:':'Status'
             }

  if msg.message.startswith('H: '):
    parts = msg.message.split()
    print (parts)
    key = None
    values = []

    state = {'timestamp':rospy.Time.now()}
    hb = Heartbeat()
    hb.header.stamp = state['timestamp']
    for p in parts:
      if p in decoder.keys():
        if key is not None:
          kv = HBKeyValue()
          kv.key = decoder[key]
          kv.value = ', '.join(values)
          hb.values.append(kv)
        key = p
        values = []
      else:
        values.append(p)
        if key == 'H:' and not 'heading' in state:
          state['heading'] = float(p)
        if key == 'D:' and not 'depth' in state:
          state['depth'] = float(p)

    if key is not None:
      kv = HBKeyValue()
      kv.key = decoder[key]
      kv.value = ', '.join(values)
      hb.values.append(kv)
    heartbeat_pub.publish(hb)
    last_state = state
    



def positionCallback(msg):
  gps = GeoPoseStamped()
  gps.header = msg.header
  gps.pose.position = msg.position
  if last_state is not None:
    if 'timestamp' in last_state and rospy.Time().now() - last_state['timestamp'] < rospy.Duration(secs=60):
      if 'depth' in last_state:
        gps.pose.position.altitude = -last_state['depth']
      if 'heading' in last_state:
        yaw = math.radians(project11.nav.headingToYaw(last_state['heading']))
        quat = quaternion_about_axis(yaw, (0,0,1))
        gps.pose.orientation.x = quat[0]
        gps.pose.orientation.y = quat[1]
        gps.pose.orientation.z = quat[2]
        gps.pose.orientation.w = quat[3]
  position_pub.publish(gps)
  #print (msg)

rospy.init_node('command_bridge_sender', anonymous=False)

sms_sub = rospy.Subscriber('sms', SMS, smsCallback)
position_sub = rospy.Subscriber('position', GeoPointStamped, positionCallback)

heartbeat_pub = rospy.Publisher('project11/heartbeat', Heartbeat, queue_size=1)
position_pub = rospy.Publisher('nav/position', GeoPoseStamped, queue_size=1)

rospy.spin()
