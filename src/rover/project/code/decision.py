import numpy as np
from rover_sates import *
from state_machine import *



# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover, machine):

    if Rover.nav_angles is not None:
        machine.run()
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover
