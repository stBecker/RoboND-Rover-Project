# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
import time

import numpy as np


def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if (time.time() - Rover.second_counter) > 1:
        Rover.second_counter = time.time()
        print(Rover.mode)

        # import pickle
        # with open("data.pickle", 'wb') as f:
        #     pickle.dump(Rover, f)

        # for k, v in Rover.__dict__.items():
        #     if isinstance(v, np.ndarray) or isinstance(v, list):
        #         continue
        #     print(k, "\t\t", v)

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is None:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        return Rover

    if Rover.picking_up:
        # do nothing
        Rover.steer = 0
        Rover.brake = 0
        Rover.throttle = 0
        Rover.picking = False
        return Rover

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = 'stop'
        return Rover

    if Rover.near_sample:
        Rover.throttle = 0
        Rover.brake = 100
        Rover.steer = 0
        return Rover

    if sample_detected(Rover):
        Rover.mode = "approach_sample"

    elif path_is_blocked(Rover):
        Rover.mode = "stop"

    elif (not Rover.mode == "stop" or abs(Rover.steer) > 2) and is_stuck(Rover):
        Rover.mode = "reverse"
        Rover.last_update_time = time.time()

    action = {
        "forward": forward_v1,
        "stop": stop_v1,
        "reverse": reverse,
        "approach_sample": approach_sample,
    }[Rover.mode]
    Rover = action(Rover)

    steer_diff = abs(Rover.steer - Rover.previous_steer)
    if steer_diff > Rover.steer_set:
        # smooth out steering angle
        if Rover.steer >= Rover.previous_steer:
            # turning left
            Rover.steer = Rover.previous_steer + Rover.steer_set
        else:
            Rover.steer = Rover.previous_steer - Rover.steer_set

    Rover.previous_steer = Rover.steer

    return Rover


def reverse(Rover):
    Rover.brake = 0
    Rover.throttle = -1

    # todo: try to turn as well if still no movement

    if time.time() - Rover.last_update_time > 3:
        Rover.last_update_time = time.time()
        Rover.mode = "forward"
        Rover.throttle = Rover.throttle_set

    return Rover


def sample_detected(Rover):
    if Rover.rock_angles.size > 7:
        return True
    return False


def approach_sample(Rover):
    if Rover.rock_angles.size == 0:
        # lost visual contact with sample
        Rover.mode = "forward"
        return Rover

    angles_in_deg = np.rad2deg(Rover.rock_angles)
    rock_direction = np.mean(angles_in_deg)

    # slow down and approach
    if Rover.vel > 0.8:
        Rover.brake = 0.5
        Rover.throttle = 0
    else:
        Rover.brake = 0
        Rover.throttle = 0.3

    Rover.steer = np.clip(rock_direction, -15, 15)
    # print(Rover.steer)

    return Rover


def path_is_blocked(Rover):
    angles_in_deg = np.rad2deg(Rover.obstacles_angles)
    # consider only obstacles in front of the rover
    relevant_angles = (angles_in_deg >= -10) & (angles_in_deg <= 10)
    relevant_dist = (Rover.obstacles_dists > 8) & (Rover.obstacles_dists < 30)
    relevant = relevant_angles & relevant_dist
    dist = Rover.obstacles_dists[relevant]
    angles = angles_in_deg[relevant]

    if angles.size > 50:
        return True

    return False


def is_stuck(Rover):
    if time.time() - Rover.last_update_time > 15:
        # how far have we moved within the last x seconds
        xpos = Rover.last_pos[0] - Rover.pos[0]
        ypos = Rover.last_pos[1] - Rover.pos[1]
        dist = np.sqrt(xpos ** 2 + ypos ** 2)

        # update values
        Rover.last_update_time = time.time()
        Rover.last_pos = Rover.pos

        # print("dist to POS: ",dist)
        if dist < 15:
            return True

    return False


def forward_v1(Rover):
    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        if Rover.vel < Rover.max_vel:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        else:  # Else coast
            Rover.throttle = 0
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

    return Rover


def stop_v1(Rover):
    # If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
    # If we're not moving (vel < 0.2) then do something else
    elif Rover.vel <= 0.2:
        # Now we're stopped and we have vision data to see if there's a path forward
        if len(Rover.nav_angles) < Rover.go_forward or path_is_blocked(Rover):
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            # Rover.steer = -15  # Could be more clever here about which way to turn
            # steer towards more promising direction
            angles_in_deg = np.rad2deg(Rover.nav_angles)
            if Rover.previous_steer in (-15, 15):
                # already committed to a steering direction, keep going
                Rover.steer = Rover.previous_steer
            else:
                if np.sum(angles_in_deg >= 0) > np.sum(angles_in_deg < 0):
                    Rover.steer = 15
                    Rover.previous_steer = 15
                else:
                    Rover.steer = -15
                Rover.previous_steer = -15

        # If we're stopped but see sufficient navigable terrain in front then go!
        else:
            # Set throttle back to stored value
            Rover.throttle = Rover.throttle_set
            # Release the brake
            Rover.brake = 0
            # Set steer to mean angle
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            Rover.mode = 'forward'

    return Rover
