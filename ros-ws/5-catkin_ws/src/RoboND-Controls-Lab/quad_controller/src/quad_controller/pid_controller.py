#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 15 20:26:32 2017

@author: juan
"""


class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_windup=10, alpha=1., u_bounds=[float('-inf'), float('inf')]):
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        self.r_ = 0.0
        self.max_windup_ = float(max_windup)

        # Set alpha for derivative filter smoothing factor
        self.alpha = float(alpha)

        # Setting control effort saturation limits
        self.umin = u_bounds[0]
        self.umax = u_bounds[1]

        # Store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

        # Control effort history
        self.u_p = [0]
        self.u_i = [0]
        self.u_d = [0]

    def reset(self):
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0
        self.last_last_error_ = 0
        self.last_windup_ = 0.0

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKI(self, ki):
        self.ki_ = float(ki)

    def setKD(self, kd):
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        self.max_windup_ = int(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            # Delta time is zero
            return 0

        # Calculate the error
        error = self.set_point_ - measured_value

        # Sum the errors
        self.error_sum_ += error * delta_time

        # Set the last_timestamp_
        self.last_timestamp_ = timestamp

        # Find delta_error
        delta_error = error - self.last_error_

        # Update the past error
        self.last_error_ = error

        # Address max windup
        ########################################
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_
        ########################################

        # Proportional error
        p = self.kp_ * error

        # Integral error
        i = self.ki_ * self.error_sum_

        # Recalculate the derivative error here incorporating
        # derivative smoothing!
        ########################################
        d = self.alpha * (self.kd_ * delta_error / delta_time) + \
            (1 - self.alpha) * self.last_error_
        ########################################

        # Set the control effort
        u = p + i + d

        # Enforce actuator saturation limits
        ########################################
        if u > self.umax:
            u = self.umax
        elif u < self.umin:
            u = self.umin
        ########################################

        # Here we are storing the control effort history for post control
        # observations.
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)

        return u
