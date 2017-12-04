import numpy as np
import time
import copy


# This is where you can build a decision tree for determining throttle, brake and steer

def rad2deg(rad):
    return np.array(rad * 180 / np.pi)


def get_polar_points(Rover):
    if Rover.nav_dists is not None:
        dist = Rover.nav_dists
    else:
        dist = 0
    if Rover.nav_angles is not None:
        ang = Rover.nav_angles
    else:
        ang = 0
    Rover.nav_angles
    return np.array((dist, ang)).T


def get_frontal_distance(polar_points, arc=10):
    central_view = [d for d, a in polar_points if rad2deg(a) < abs(arc)]
    return np.array(central_view)


def get_wall_distance(polar_points, arc=15):
    wall_d = [d for d, a in polar_points if rad2deg(a) < -arc]
    wall_i = [d for d, a in polar_points if rad2deg(a) > arc]
    return np.array(wall_i) if len(wall_i) else np.array(0),\
        np.array(wall_d) if len(wall_d) else np.array(0)


def get_near_periferics(polar_points, alpha):
    near_angles = [a for d, a in polar_points if d < alpha]
    return np.array(near_angles)


def detec_rock(Rover):
    global r_d_0
    global r_a_0
    if len(Rover.rock_angles):
        rock_p = np.array((Rover.rock_dists, Rover.rock_angles)).T
        r_d_0 = copy.copy(rock_p[:, 0].mean())
        r_a_0 = copy.copy(rad2deg(rock_p[:, 1].mean()))
        return rock_p
    else:
        return None


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


def counter_state(Rover, sta_change=True):
    Rover.state_counter += 1
    if Rover.state_counter >= 100:
        Rover.state_counte = 0
        if sta_change:
            Rover.mode = Rover.states[np.random.randint(0, len(Rover.states))]
        return True
    else:
        return False


# commands based on the output of the perception_step() function
turn = True
# r_a_0 = Rover.rock_dists
# r_d_0 = Rover.Roc


def decision_step(Rover):
    global turn
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # detec_steer(Rover)
    # flag = Rover.flag_direction
    # Example:
    # Check if we have vision data to make decisions with
    print Rover.mode
    if Rover.nav_angles is not None:

        # Check for Rover.mode status
        nav_data = get_polar_points(Rover)
        # mean_dir = np.mean(Rover.nav_angles)
        # desv = np.sqrt(Rover.nav_angles.var())
        mean_dir = np.mean(get_near_periferics(nav_data, 15))
        desv = np.sqrt(get_near_periferics(nav_data, 15).var())
        mean_tita = rad2deg(mean_dir)
        tita = rad2deg(mean_dir + desv)
        tita2 = rad2deg(mean_dir - desv)
        # rock = detec_rock(Rover)
        ai, ad = side_areas(Rover)
        fd = get_frontal_distance(nav_data, 25).mean()
        wd = get_wall_distance(nav_data, 20)
        wdi = wd[0].mean()
        wdd = wd[1].mean()
        r_a_0 = Rover.rock_angles
        r_d_0 = Rover.rock_dists
        if Rover.mode == 'forward':
            # Rover.max_vel =2
            if abs(Rover.throttle) >= 2:
                Rover.throttle = 0
            print Rover.mode
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                print 'Area Libre ', len(Rover.nav_angles)
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    print 'Acc: ', Rover.throttle, 'Vel: ', Rover.vel
                    if (Rover.throttle > 0) & (Rover.vel == 0):
                        # Atorado
                        print 'Atorado'
                        Rover.mode = 'Atorado'
                # elif Rover.vel >= Rover.max_vel:
                #     Rover.throttle = 0
                #     Rover.brak = Rover.brake_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                print 'tita: ', tita, ' media: ', mean_tita
                print 'front d: ', fd, 'wdi', wdi, 'wdd', wdd
                print 'Area Libre ', len(Rover.nav_angles), 'ai:', ai, 'ad:', ad
                print 'Rock', r_d_0, r_d_0
                if abs(mean_tita) < 2:  #
                    print ' derecho'
                    Rover.steer = 5
                else:
                    if abs(tita) <= 15:  # tita
                        print 'Rago Tita <= 10'
                        if abs(tita) >= 4:  # me choco la pared
                            print '+'
                            if ai >= 0.4:
                                Rover.steer = np.int(tita)  # izq
                            else:
                                Rover.steer = np.int(tita)
                            print 'steer: ', Rover.steer
                        else:
                            Rover.steer = -np.int(3)
                            print 'steer: ',  Rover.steer

                    else:  # tits > 10
                        if tita > 15:
                            print 'tita > 10'
                            # print 'fronta d: ', get_frontal_distance(nav_data, 30).mean()
                            if fd > 10:
                                if ai > 0.4:
                                    Rover.steer = np.clip(
                                        np.int(tita / 3), -15, 15)
                                else:
                                    Rover.steer = - \
                                        np.clip(np.int(tita / 3), -15, 15)
                                print 'steer: ',  Rover.steer
                            else:
                                Rover.mode = 'stop'
                        elif tita < -15:
                            print 'tita < -10'
                            # print 'fronta d: ', get_frontal_distance(nav_data, tita).mean()
                            if fd > 10:
                                if ai > 0.3:
                                    Rover.steer = - \
                                        np.clip(np.int(tita / 3), -15, 15)
                                else:
                                    Rover.steer = 0
                                    # Rover.steer = np.clip((tita/3), -15, 15)
                                print 'steer: ',  Rover.steer
                            else:
                                Rover.mode = 'stop'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif (len(Rover.nav_angles) < Rover.stop_forward) or (fd <= 15):
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                print Rover.mode, 'nav_angles < stop_forward'
            #
            if Rover.rock_dists > 0:
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.throttle = 0
                Rover.mode = 'stop'

                # raw_input()
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print 'Area Libre ', len(Rover.nav_angles), 'ai:', ai, 'ad:', ad
            print 'front d: ', fd, 'wdi', wdi, 'wdd', wdd

            # If we're in stop mode but still moving keep braking
            if abs(Rover.vel) > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif abs(Rover.vel) <= 0.2:
                print 'steady'
                # Now we're stopped and we have vision data to see if there's a path forward
                if Rover.rock_dists > 0:
                    Rover.brake = 0
                    Rover.mode = 'Rock'
                elif (len(Rover.nav_angles) < Rover.go_forward / 3) or \
                        (fd < 8):
                    print 'pa tra'
                    Rover.brake = 0
                    Rover.throttle = -0.1
                    # Release the brake to allow turning
                elif (len(Rover.nav_angles) < Rover.go_forward / 2)\
                 or (ai < 0.25) or (wdd < 8) or (wdi < 8):
                    print 'rotate'
                    # elif len(Rover.nav_angles) < Rover.go_forward:
                    print 'fd: ', fd
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -5  # der
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif (len(Rover.nav_angles) >= Rover.go_forward) &\
                        (fd > 10) & (wdd > 10) & (wdi > 10):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(mean_tita, -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'Rock':
            # Rover.throttle = 0.02
            # time.sleep(1)
            print 'Vio una piedra'
            # Rover.max_vel = 0.5
            print 'Rock: ', r_d_0, r_a_0

            Rover.throttle = 0
            print 'shearching'
            # rock = detec_rock(Rover)
            # if rock is not None:
            # rock_dist = rock[:, 0].mean()
            # rock_ang = rad2deg(rock[:, 1].mean())

            print 'rock d: ', r_d_0, 'rock a: ', r_a_0
            Rover.brake = 0
            if Rover.near_sample == 1:
                Rover.rock_dists = 0
                Rover.rock_angles = 0
                Rover.mode = 'stop'

            elif r_d_0 <= 10:  # cerca
                print 'cerca'
                if abs(r_a_0) < 10:
                    Rover.steer = 0
                else:
                    Rover.steer = np.clip(np.int(r_a_0), -2, 2)
                    if (Rover.vel <= 0.2) & (r_d_0 == 0):
                        Rover.throttle = 0.1
                    else:
                        Rover.throttle = 0
            elif (abs(r_a_0) >= 35) & (r_d_0 > 10):
                print ' lejos y abierta'
                Rover.steer = np.clip(np.array(r_a_0, dtype='int'), -2, 2)
            elif (abs(r_a_0) < 45) & (r_d_0 > 10):
                print ''
                if Rover.vel >= 0.3:
                    Rover.throttle = 0
                else:
                    Rover.throttle = 0.1
            # else:
            #     print 'Rock es none'
            #     # Rover.brake = 0
            #     Rover.steer = np.clip(np.array(r_a_0, dtype='int'), -2, 2)
            #     # Rover.throttle = 0.01
                    # Rover.mode = ''
                    # if last:
                    # else:
                    #     Rover.steer = np.clip(-5,-10,10)

        elif Rover.mode == 'Atorado':
            Rover.throttle = 0
            # Rover.state_counter +=1
            # if Rover.state_counter > 10:
            #     turn = not turn
            #     Rover.state_counter = 0
            Rover.steer = -15
            print len(Rover.nav_angles)
            if(len(Rover.nav_angles) >= Rover.go_forward / 2):
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = np.clip(np.int(mean_tita), -15, 15)
                Rover.mode = 'forward'
            #     Rover.steer = 15
            # Rover.mode = 'foward'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        print 'la nada'
        counter_state(Rover)
        Rover.steer = 0
        Rover.brake = 0
        Rover.throttle = -Rover.throttle_set
        if rock is not None:
            Rover.mode = 'Rock'

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    # time.sleep(0.5)
    # If in a state where want to pickup a rock send pickup command

    return Rover
