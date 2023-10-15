#!/usr/bin/env python3

import rospy
from project11_msgs.msg import AcousticLayer
from std_msgs.msg import String, Float32

command_template = 'SMS:2509|GOAL1 2 {0:0.3f} {1:0.3f}; WAIT 1800'

last_command_time = None
last_command_depth = None

last_depth = None
last_depth_time = None

minimum_speed = 0.01
maximum_speed = 0.35
maximum_speed_depth_difference = 20.0

def depthCallback(msg: Float32):
  global last_depth, last_depth_time
  last_depth = msg.data
  last_depth_time = rospy.Time.now()

def layerCallback(msg: AcousticLayer):
  global last_command_time, last_command_depth
  now = rospy.Time.now()
  if last_command_time is None or last_command_time < now-rospy.Duration(55) or (last_depth_time is not None and last_depth_time > last_command_time):
    last_command_depth = msg.slices[-1].center_of_mass_range
    speed = minimum_speed
    #print('now', rospy.Time.now(),'last depth', last_depth, 'time', last_depth_time)
    if last_depth_time is not None and last_depth_time > rospy.Time.now() - rospy.Duration(90.0):
      depth_difference = abs(last_command_depth-last_depth)
      speed = maximum_speed*depth_difference/maximum_speed_depth_difference
      speed = min(maximum_speed, max(speed, minimum_speed))
      # if abs(last_command_depth-last_depth) > 5.0:
      #   speed = 0.25
      # elif abs(last_command_depth-last_depth) > 2.0:
      #   speed = 0.1
      # elif abs(last_command_depth-last_depth) > 1.0:
      #   speed = 0.05
    command = String()
    command.data = command_template.format(last_command_depth, speed)
    command_pub.publish(command)
    last_command_time = now

rospy.init_node("follow_layer")

layer_sub = rospy.Subscriber('/project11/drix_8/sensors/ek80/es70_18cd/es70_18cd_tracker/layer', AcousticLayer, layerCallback)

command_pub = rospy.Publisher('/project11/drix_8/usbl_modem/send_raw', String, queue_size=1)

depth_sub = rospy.Subscriber('/project11/mesobot/depth', Float32, depthCallback)

rospy.spin()

