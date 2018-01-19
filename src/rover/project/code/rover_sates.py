# -*- coding: utf-8 -*-
from helper import *
import time


class Stop(object):
    def __init__(self, rover, brake=5):
        self.rover = rover
        self.brake = brake
        self.rover.steer = 0
        self.start_time = self.rover.total_time

    def __str__(self):
        return 'Stop'

    def delay(self, sec):
        if self.start_time == 0:
            self.start_time = self.rover.total_time
        delta = self.rover.total_time - self.start_time
        if delta <= sec:
            # print 'stabilizing...', delta
            return False
        else:
            self.start_time = 0
            return True


    def run(self):
        self.rover.brake = self.brake
        self.rover.throttle = 0.0
        self.rover.steer = 0

    def next(self):
        self.rover.throttle = 0.0
        self.rover.steer = 0
        if abs(self.rover.vel) < 0.1:
            if self.delay(3):
                self.rover.brake = 0.0
                if self.rover.go_home:
                    return ReturnHome(self.rover)
                else:
                    return SearchClearPath(self.rover)
            else:
                return self
        else:
            return self


class Go(object):
    def __init__(self, rover, throttle=0.1):
        self.rover = rover
        self.throttle = throttle
        self.bearing = 0
        self.nav_data = None
        self.front_area = 0
        self.start_time = 0

    def __str__(self):
        return 'Go'

    def delay(self, sec):
        if self.start_time == 0:
            self.start_time = self.rover.total_time
        delta = self.rover.total_time - self.start_time
        if delta <= sec:
            # print 'stabilizing...', delta
            return False
        else:
            self.start_time = 0
            return True

    def run(self):
        self.rover.brake = 0
        self.rover.throttle = self.throttle

    def update_sterring(self):
        self.nav_data = get_polar_points(self.rover)
        mean_dir = rad2deg(np.mean(get_near_periferics(self.nav_data, 100)))
        desv = rad2deg(np.sqrt(get_near_periferics(self.nav_data, 100).var()))
        AI, AD = side_areas(self.rover)
        if AI > 0.45:
            self.bearing = np.int_(mean_dir)
            self.rover.steer = np.clip(self.bearing, -15, 15)
        elif AI > 0.30:
            self.bearing = np.int_(mean_dir + 0.8 * desv)
            self.rover.steer = np.clip(self.bearing, -2, 15)
        elif AI < 0.15:
            self.bearing = np.int_(mean_dir + 0.5 * desv)
            self.rover.steer = np.clip(self.bearing, -12, 12)
        else:
            self.bearing = 0
            self.rover.steer = self.bearing

    def check_area_stop(self):
        if len(self.rover.nav_angles) > self.rover.stop_forward:
            return True
        else:
            return False

    def check_vel_max(self):
        if self.rover.vel >= self.rover.max_vel:
            self.rover.throttle = 0.0
        else:
            self.rover.throttle = self.rover.throttle_set

    def stuck(self):
        if self.rover.vel < 0.05:
            if self.delay(0.2):
                if self.rover.throttle > 0:
                    return True
                else:
                    return False
            return False
        else:
            return False

    def check_rock_sight(self):
        if distance_to_rock(self.rover) > 0:
            return True
        else:
            return False

    def next(self):
        print 'area: ', len(self.rover.nav_angles)
        print 'fron area:', self.front_area
        if self.check_rock_sight():
            self.rover.rock_detected = True
            return Stop(self.rover, brake=10)
        if self.check_area_stop():  # and is_obstacle_ahead(self.rover) ==
            self.check_vel_max()
            self.update_sterring()
            self.front_area = is_obstacle_ahead(self.rover, range=20, bearing=self.bearing)
            if self.front_area > 50:
                print 'fron area:', self.front_area
                return Stop(self.rover)
            if self.stuck():
                return Stuck(self.rover)
            else:
                return self
        else:
            return Stop(self.rover)


class SearchClearPath(object):
    def __init__(self, rover, turn='right'):
        self.rover = rover
        self.turn_direction = turn
        self.iteration = 0

    def __str__(self):
        return 'SearchClearPath'

    def run(self):
        self.rover.brake = 0.0

    def update_turn(self):
        if self.turn_direction is 'right':
            self.rover.steer = -15
        elif self.turn_direction is 'left':
            self.rover.steer = 15
        self.iteration += 1

    def toogle_turn(self):
        if self.turn_direction is 'right':
            self.turn_direction = 'left'
        elif self.turn_direction is 'left':
            self.turn_direction = 'right'

    def next(self):
        print 'iter: ', self.iteration
        self.update_turn()
        AI, AD = side_areas(self.rover)
        print 'area: ', len(self.rover.nav_angles)
        print 'AI:', AI
        if self.rover.rock_detected:
            return Rock(self.rover)
        else:
            if len(self.rover.nav_angles) >= self.rover.go_forward:
                if is_obstacle_ahead(self.rover, 25, 0, arc=15) > 10:
                    return self
                else:
                    if AI < 0.40:
                        return self
                    else:
                        self.iteration = 0
                        self.rover.steer = 0
                        return Go(self.rover)
            # elif self.iteration < 100:
            #     return self
            elif self.iteration == 500:
                # raw_input()
                # self.iteration += 1
                self.toogle_turn()
                return self
            elif self.iteration >= 1000:
                # raw_input()
                # self.iteration += 1
                # if self.iteration >= 200:
                    # raw_input()
                self.iteration = 0
                return Go(self.rover)
                # else:
                    # return self
            else:
                # self.iteration +=1
                return self


class Stuck(object):
    def __init__(self, rover):
        self.rover = rover
        self.times = 0

    def __str__(self):
        return 'Stuck'

    def check_vel_max(self):
        if self.rover.vel < -self.rover.max_vel:
            return False
        else:
            return True

    def run(self):
        self.rover.steer = 0
        self.rover.throttle = 0

    def next(self):
        if self.rover.picking_up:
            return Stop(self.rover)
        self.times += 1
        if self.times >=1:
            if self.times >= 40:
                self.rover.throttle = 0.0
                self.times = 0
                return Stop(self.rover)
            else:
                if self.check_vel_max():
                    self.rover.throttle = -0.1
                else:
                    self.rover.throttle = 0
                return self
        return self

class Rock(Go):
    def __init__(self, rover):
        self.rover = rover
        self.distance = 0
        self.angle = 0
        self.iteration = 30
        self.bearing = 0
        self.start_time = 0

    def __str__(self):
        return 'Rock'

    def update_rock_data(self):
        self.distance = distance_to_rock(self.rover)
        if self.distance != 0:
            self.angle = rad2deg(np.mean(self.rover.rock_angles))

    def check(self):
        # If in a state where want to pickup a rock send pickup command
        if self.rover.near_sample:
            self.rover.throttle = 0
            self.rover.brake = self.rover.brake_set
            if self.rover.vel == 0 and not self.rover.picking_up:
                self.rover.send_pickup = True
                while self.rover.picking_up:
                    print 'picking up'
                self.rover.rock_detected = False
                self.rover.max_vel = 1
                return True
            else:
                self.rover.brake = self.rover.brake_set
                return False
        else:
            return False

    def putting_rock_infront(self):
        if abs(self.angle) > 25:
            self.bearing = self.angle
            self.rover.steer = np.clip(self.bearing, -15, 15, out=None)
        else:
            self.rover.steer = 0

    def delay(self, sec):
        if self.start_time == 0:
            self.start_time = self.rover.total_time

        delta = self.rover.total_time - self.start_time
        while delta - self.start_time < sec:
            # print 'stabilizing...', delta
            return False
        else:
            self.start_time = 0
            return True

    def go(self):
        if self.rover.vel > self.rover.max_vel:
            self.rover.throttle = 0
        else:
            self.rover.throttle = 0.1

    def run(self):
        # self.delay(100)
        self.rover.max_vel = 0.5
        self.update_rock_data()
        # self.putting_rock_infront()
        # self.first_angle = self.angle

    def next(self):
        self.update_rock_data()
        if self.distance == 0:
            self.iteration += 1
            self.rover.steer = np.clip(self.bearing,-15,15)
            self.go()
            if self.iteration >= 15:
                self.rover.max_vel = 1.5
                return Go(self.rover, throttle=0.1)
            else:
                return self
        else:
            self.iteration = 0
            print 'distance:', self.distance
            print 'angle: ', self.angle
            self.go()
            self.putting_rock_infront()
            if self.check():
                if self.rover.samples_to_find == 0:
                    return s(self.rover)
                else:
                    if np.sign(self.bearing):
                        turn = 'right'
                    else:
                        turn = 'left'
                    return SearchClearPath(self.rover, turn)
            else:
                if self.stuck():
                    self.putting_rock_infront()
                    return Stuck(self.rover)
                else:
                    return self


class ReturnHome(Go):
    def __init__(self, rover):
        self.rover = rover
        self.home = self.rover.pos
        self.front_area = 0

    def __str__(self):
        return 'ReturnHome'

    def bearing_to_home_position(self):
        x = self.rover.pos[0] - self.home[0]
        y = self.rover.pos[1] - self.home[1]
        bearing = rad2deg(np.arctan2(y, x))
        return bearing

    def update_sterring(self):
        min_nav_angle = rad2deg(np.min(self.rover.nav_angles)) + 45
        max_nav_angle = rad2deg(np.max(self.rover.nav_angles)) - 45
        min_obs_angle = rad2deg(np.min(self.rover.nav_angles)) + 45
        max_obs_angle = rad2deg(np.max(self.rover.nav_angles)) - 45

        min_angle = max(min_nav_angle, min_obs_angle)
        max_angle = min(max_nav_angle, max_obs_angle)

        self.rover.steer = np.clip(self.bearing_to_home_position(),
                                   min_angle, max_angle)
        self.front_area = is_obstacle_ahead(self.rover, range=30, bearing=self.bearing_to_home_position())

    def run(self):
        pass

    def next(self):
        if self.rover.samples_to_find == 0:
            self.rover.go_home = True
            print 'area: ', len(self.rover.nav_angles)
            print 'front area:', self.front_area
            if self.check_area_stop():  # and is_obstacle_ahead(self.rover) ==
                self.update_sterring()
                self.check_vel_max()
                if self.front_area > 100:
                    return Stop(self.rover)
                if self.stuck():
                    return Stuck(self.rover)
                else:
                    return self
            else:
                return Stop(self.rover)
        else:
            return Go(self.rover, throttle=0.1)
