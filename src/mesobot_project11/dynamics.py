#!/usr/bin/env python3

import project11.geodesic
import math

class MesobotDynamics:
    """ Simulate Mesobot motion resulting from input commands.
    """

    def __init__(self):
        self.depth_change_rate = .2 # m/s
        self.heading_change_rate = 0.2 # rad/sec
        self.maximum_forward_speed = 0.5 # m/s
        self.last_time = None

        self.clearCommands()

    def clearCommands(self):
        self.target_heading = None
        self.target_depth = None
        self.forward_speed_ratio = 0.0
        self.forward_speed_start_time = None
        self.forward_speed_duration = 0

    def reset(self, lat, lon, depth, heading, timestamp):
        self.latitude = lat
        self.longitude = lon
        self.depth = depth
        self.heading = heading

        self.clearCommands()
        self.last_time = timestamp

    def iterate(self, timestamp):
        if self.last_time is None:
            self.last_time = timestamp
        else:
            dt = timestamp - self.last_time
            if dt <= 0: # time has not advanced
                return False
            
            if self.target_depth is not None and self.depth != self.target_depth:
                potential_depth_change = self.depth_change_rate*dt
                depth_diff = self.target_depth - self.depth
                if abs(depth_diff) > potential_depth_change:
                    if depth_diff < 0:
                        self.depth -= potential_depth_change
                    else:
                        self.depth += potential_depth_change
                else:
                    self.depth = self.target_depth

            if self.forward_speed_ratio > 0:
                if timestamp > self.forward_speed_start_time+self.forward_speed_duration:
                    self.forward_speed_ratio = 0.0
                else:
                    moved = dt*self.maximum_forward_speed*self.forward_speed_ratio
                    lon_rad, lat_rad = project11.geodesic.direct(math.radians(self.longitude), math.radians(self.latitude), self.heading, moved)
                    self.latitude = math.degrees(lat_rad)
                    self.longitude = math.degrees(lon_rad)

            if self.target_heading is not None and self.heading != self.target_heading:
                potential_heading_change = self.heading_change_rate*dt
                heading_difference = self.target_heading - self.heading
                while heading_difference < -math.pi:
                    heading_difference += 2*math.pi
                while heading_difference > math.pi:
                    heading_difference -= 2*math.pi
                print("heading difference",heading_difference)
                if abs(heading_difference) > potential_heading_change:
                    if heading_difference < 0:
                        self.heading -= potential_heading_change
                    else:
                        self.heading += potential_heading_change
                else:
                    self.heading = self.target_heading
                while self.heading < 0.0:
                    self.heading += 2*math.pi
                while self.heading > 2*math.pi:
                    self.heading -= 2*math.pi
                print("new heading", self.heading)



            self.last_time = timestamp
            return True
            

    def command(self, cmd, timestamp):
        parts = cmd.split()
        if parts[0] == 'GOAL1':
            dof = parts[1] # degree of freedom
            if dof == '2': # down
                self.target_depth = float(parts[2])
            if dof == '5': # yaw
                self.target_heading = float(parts[2])
                print("target heading:",self.target_heading)
        if parts[0] == 'DRIVE':
            self.forward_speed_ratio = float(parts[1])
            self.forward_speed_duration = float(parts[2])
            self.forward_speed_start_time = timestamp
            print("got drive cmd","ratio", self.forward_speed_ratio, "duration", self.forward_speed_duration, "start time", self.forward_speed_start_time)
        if parts[0] == 'DEPTH_OFFSET':
            self.target_depth = self.depth + float(parts[1])

    def status(self):
        return 'H: '+str(math.degrees(self.heading))+" D: "+str(self.depth)