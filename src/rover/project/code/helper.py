import numpy as np


def rad2deg(rad):
    return (rad * 180.0) / np.pi


def deg2rad(deg):
    return (deg * np.pi) / 180.0


def distance_to_rock(Rover):
    if Rover.rock_dists is not None:
        if len(Rover.rock_dists):
            return np.min(Rover.rock_dists)
        else:
            return 0
    else:
        return 0


def is_obstacle_ahead(Rover, range=25, bearing=0, arc=15):
    idx_in_front = np.where((np.abs(Rover.obs_angles - bearing) < deg2rad(arc))
                            & (Rover.obs_dists < range))[0]
    return len(idx_in_front)


def get_polar_points(Rover):
    dist = Rover.nav_dists
    ang = Rover.nav_angles
    return np.array((dist, ang)).T

def get_frontal_distance(polar_points, arc=10):
    central_view = [d for d, a in polar_points if rad2deg(abs(a)) < abs(arc)]
    return np.array(central_view)

def get_near_periferics(polar_points, alpha):
    near_angles = [a for d, a in polar_points if d < alpha]
    return np.array(near_angles)

def side_areas(Rover):
    i = 0
    d = 0
    if len(Rover.nav_angles) != 0:
        for a in Rover.nav_angles:
            if a > 0:
                i += 1
            else:
                d += 1
        return (1.0 * i / len(Rover.nav_angles), 1.0 * d / len(Rover.nav_angles))
    else:
        return (0, 0)
