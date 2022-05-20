#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32
from sonardyne_msgs.msg import SMS

def rawCallback(msg):
  if '|' in msg.data:
    message = msg.data.split('|',1)[1].strip()
    #print (message)
    parts = message.split()
    if len(parts) == 4 and parts[0] == 'CTD:':
      value = Float32()
      value.data = float(parts[1])
      c_pub.publish(value)
      value.data = float(parts[2])
      t_pub.publish(value)
      value.data = -float(parts[3])
      d_pub.publish(value)


def sendTest(event):
  sms = SMS()
  sms.address = '2509'
  sms.message = 'TEST'
  sms_pub.publish(sms)

rospy.init_node('ctd', anonymous=False)

sms_sub = rospy.Subscriber('raw', String, rawCallback)
sms_pub = rospy.Publisher('send_sms', SMS, queue_size=1)
t = rospy.Timer(rospy.Duration(19.5), sendTest)

c_pub = rospy.Publisher('ctd/c', Float32, queue_size=1)
t_pub = rospy.Publisher('ctd/t', Float32, queue_size=1)
d_pub = rospy.Publisher('ctd/d', Float32, queue_size=1)

rospy.spin()
