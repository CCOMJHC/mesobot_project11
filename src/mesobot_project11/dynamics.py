#!/usr/bin/env python3

import project11.geodesic

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
            
            if self.depth != self.target_depth:
                potential_depth_change = self.depth_change_rate*dt
                depth_diff = self.target_depth - self.depth
                if abs(depth_diff) > potential_depth_change:
                    if depth_diff < 0:
                        self.depth -= potential_depth_change
                    else:
                        self.depth += potential_depth_change
                else:
                    self.depth = self.target_depth


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
        if parts[0] == 'DRIVE':
            self.forward_speed_ratio = float(parts[1])
            self.forward_speed_duration = float(parts[2])
            self.forward_speed_start_time = timestamp
        if parts[0] == 'DEPTH_OFFSET':
            self.target_depth = self.depth + float(parts[1])

