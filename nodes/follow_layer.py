#!/usr/bin/env python3

import rospy
from project11_msgs.msg import AcousticLayer
from std_msgs.msg import String

command_template = 'SMS:2509|GOAL1 2 {0:0.3f}; WAIT 1800'

last_command_time = None
last_command_depth = None

def layerCallback(msg: AcousticLayer):
  global last_command_time, last_command_depth
  now = rospy.Time.now()
  if last_command_time is None or last_command_time < now-rospy.Duration(55):
    last_command_depth = msg.slices[-1].center_of_mass_range
    command = String()
    command.data = command_template.format(last_command_depth)
    command_pub.publish(command)
    last_command_time = now

rospy.init_node("follow_layer")

layer_sub = rospy.Subscriber('/project11/drix_8/sensors/ek80/es70_18cd/single_layer_tracker/layer', AcousticLayer, layerCallback)

command_pub = rospy.Publisher('/project11/drix_8/usbl_modem/send_raw', String, queue_size=1)

rospy.spin()

