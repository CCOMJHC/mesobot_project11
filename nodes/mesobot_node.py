#!/usr/bin/env python3

import rospy

from sonardyne_msgs.msg import SMS
from geographic_msgs.msg import GeoPointStamped
from geographic_msgs.msg import GeoPoseStamped
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue

def smsCallback( msg):
  print (msg)

def positionCallback(msg):
  print (msg)

rospy.init_node('command_bridge_sender', anonymous=False)

sms_sub = rospy.Subscriber('sms', SMS, smsCallback)
position_sub = rospy.Subscriber('position', GeoPointStamped, positionCallback)

heartbeat_pub = rospy.Publisher('project11/heartbeat', Heartbeat, queue_size=1)
position_pub = rospy.Publisher('nav/position', GeoPoseStamped, queue_size=1)

rospy.spin()
