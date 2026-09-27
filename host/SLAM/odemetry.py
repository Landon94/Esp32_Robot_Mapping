import numpy as np
# INSIDE_WHEEL TO INSIDE WHEEL = 4.25 inches
# TIRE_THICKNESS = 1 inch
# WHEEL_DIAMETER = 2.5 inch


WHEEL_BASE   = 0.13335    # meters
WHEEL_RADIUS = 0.03175    # meters

TICKS_PER_REV = 20

class Odemetry:

    def __init__(self):


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_timestamp_us = None

    def update(self, timestamp, right_rpm, left_rpm):

        if self.last_timestamp_us is None:
            self.last_timestamp_us = timestamp
            return self.pose()
        
        dt = (timestamp - self.last_timestamp_us) / 1000000
        self.last_timestamp_us = timestamp
        
        left_angular_veloctity = left_rpm * np.pi * 2 / 60.0
        right_angular_velocity = right_rpm * np.pi * 2 / 60.0

        left_velocity = left_angular_veloctity * WHEEL_RADIUS
        right_velocity = right_angular_velocity * WHEEL_RADIUS

        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE

        theta_mid = self.theta + angular_velocity * dt / 2

        self.x += linear_velocity * dt * np.cos(theta_mid)
        self.y += linear_velocity * dt * np.sin(theta_mid)
        self.theta += angular_velocity * dt

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        return self.pose()

    def pose(self):
        return [self.x, self.y, self.theta]