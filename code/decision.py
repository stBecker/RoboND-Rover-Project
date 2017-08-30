import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
import time

from perception import to_polar_coords


def forward_decision(Rover, updated):
    # how far can we see
    if np.mean(Rover.nav_dists) > 80:
        Rover.throttle_set = 0.5
        Rover.max_vel = 4
    elif np.mean(Rover.nav_dists) > 30:
        Rover.throttle_set = 0.2
        Rover.max_vel = 2
    else:
        Rover.throttle_set = 0.1
        Rover.max_vel = 1

    # if len(Rover.nav_dists) > 0 and np.min(Rover.nav_dists) > 5:
    #     Rover.throttle = 0
    #     Rover.steer = 15
    #     return Rover

    angles_in_degree = Rover.nav_angles * 180 / np.pi

    if updated:
        # no movement for x seconds, rover is stuck
        print(sum(Rover.history[:-3]))
        # print(Rover.history[-1], Rover.pos)
        if len(Rover.history) > 3 and sum(Rover.history[-3:]) < 0.01:
            # if len(Rover.history) > 3 and Rover.history[:-1] == Rover.pos:
            Rover.throttle = 0
            Rover.steer = 0
            print("rover is stuck")

            # Rover.mode = "turn"
            if np.mean(angles_in_degree) > 0:
                # turn left
                Rover.mode = "turn_left"
                Rover.turn_yaw = Rover.yaw
                return Rover
            else:
                # turn right
                Rover.mode = "turn_right"
                Rover.turn_yaw = Rover.yaw
                return Rover

    # try to stay near a wall
    # if 0 <= np.min(angles_in_degree) <= 5:
    #     Rover.steer = 0

    # near wall
    if angles_in_degree.size > 0:
        # obstacle in the right
        if -5 < np.min(angles_in_degree) < -1:
            Rover.mode = "wall_right"
            print("wall_right")
            return Rover

        elif 1 < np.max(angles_in_degree) < 5:
            Rover.mode = "wall_left"
            print("wall_left")
            return Rover

        if sum(angles_in_degree > 0) / float(angles_in_degree.size) > 0.9:
            if sum((angles_in_degree > 0) & (angles_in_degree < 5)) > 10:
                Rover.steer = 0
            else:
                Rover.steer = np.mean(angles_in_degree)

        elif sum(angles_in_degree < 0) / float(angles_in_degree.size) > 0.9:
            if sum((angles_in_degree < 0) & (angles_in_degree > -5)) > 10:
                Rover.steer = 0
            else:
                Rover.steer = np.mean(angles_in_degree)

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
        # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

    if Rover.near_sample:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

    return Rover


def decision_step_v0(Rover, updated=False):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # record historical data every second
    # if time.time() - Rover.last_update_time > 1:
    if updated:
        Rover.history.append(Rover.vel)
        # Rover.history.append(Rover.pos)
        Rover.last_update_time = time.time()
        print(Rover.vel)
        print(Rover.mode)
        # print(Rover.__dict__)

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is None:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        return Rover

    if Rover.mode == 'turn_left':
        # if updated: print("left")
        Rover.throttle = 0
        Rover.steer = 15

        turned_angle = Rover.yaw - Rover.turn_yaw
        if turned_angle < 0:
            turned_angle += 360
        if turned_angle >= 90:
            Rover.steer = 0
            Rover.mode = 'stop'

    elif Rover.mode == 'turn_right':
        # if updated: print("right")
        Rover.throttle = 0
        Rover.steer = - 15
        turned_angle = Rover.turn_yaw - Rover.yaw
        if turned_angle < 0:
            turned_angle += 360
        if turned_angle >= 90:
            Rover.steer = 0
            Rover.mode = 'stop'

    elif Rover.mode == "wall_right":
        angles_in_degree = Rover.nav_angles * 180 / np.pi
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0

        if updated:
            # no movement for x seconds, rover is stuck
            if len(Rover.history) > 3 and sum(Rover.history[:-3]) < 0.01:
                Rover.throttle = 0
                Rover.steer = 0
                print("rover is stuck")

                # Rover.mode = "turn"
                if np.mean(angles_in_degree) > 0:
                    # turn left 45 deg
                    Rover.mode = "turn_left"
                    Rover.target_yaw = (Rover.yaw + 45) % 360
                else:
                    # turn right 45 deg
                    Rover.mode = "turn_right"
                    Rover.target_yaw = (Rover.yaw + 360 - 45) % 360

        # too far to the left
        if np.max(angles_in_degree) > 5:
            relevant_angles = angles_in_degree[angles_in_degree < 0]
            if relevant_angles.size == 0:
                Rover.steer = -10
            else:
                Rover.steer = np.mean(relevant_angles)

        # too far to the right
        elif np.max(angles_in_degree) < 1:
            relevant_angles = angles_in_degree[angles_in_degree > 0]
            if relevant_angles.size == 0:
                Rover.steer = 10
            else:
                Rover.steer = np.mean(relevant_angles)

    elif Rover.mode == "wall_left":
        angles_in_degree = Rover.nav_angles * 180 / np.pi
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0

        if updated:
            # no movement for x seconds, rover is stuck
            if len(Rover.history) > 3 and sum(Rover.history[:-3]) < 0.01:
                Rover.throttle = 0
                Rover.steer = 0
                print("rover is stuck")

                # Rover.mode = "turn"
                if np.mean(angles_in_degree) > 0:
                    # turn left 45 deg
                    Rover.mode = "turn_left"
                    Rover.target_yaw = (Rover.yaw + 45) % 360
                else:
                    # turn right 45 deg
                    Rover.mode = "turn_right"
                    Rover.target_yaw = (Rover.yaw + 360 - 45) % 360

        # too far to the right
        if np.min(angles_in_degree) < -5:
            print("too far right")
            relevant_angles = angles_in_degree[angles_in_degree > 0]
            if relevant_angles.size == 0:
                Rover.steer = 10
            else:
                Rover.steer = np.mean(relevant_angles)

        # too far to the left
        elif np.min(angles_in_degree) > -1:
            print("too far left")
            relevant_angles = angles_in_degree[angles_in_degree < 0]
            if relevant_angles.size == 0:
                Rover.steer = -10
            else:
                Rover.steer = np.mean(relevant_angles)

    # Check for Rover.mode status
    elif Rover.mode == 'forward':
        Rover = forward_decision(Rover, updated)

    # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
        if updated: print("stop")
        # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = -15  # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            if len(Rover.nav_angles) >= Rover.go_forward:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                Rover.mode = 'forward'

    if Rover.near_sample:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover


def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if (time.time() - Rover.second_counter) > 1:
        Rover.second_counter = time.time()
        print(Rover.mode)
        Rover.history.append(Rover.pos)

        for k,v in Rover.__dict__.items():
            if not isinstance(v, iter):
                print(k,v)

    # todo:
    # - avoid obstacles
    # - locate samples (perception)
    # - wall crawler

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is None:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        return Rover

    # if Rover.sample_locations_to_find is None:
    #     Rover.sample_locations_to_find = np.copy(Rover.samples_pos)

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
        # if not Rover.picking:
        #     print("remove found sample from list")
        #     dist, sample_direction, idx = find_nearest_sample(Rover)
        #     xpos = np.delete(Rover.sample_locations_to_find[0], [idx])
        #     ypos = np.delete(Rover.sample_locations_to_find[1], [idx])
        #     Rover.sample_locations_to_find = (xpos, ypos)
        Rover.mode = 'stop'
        # Rover.picking = True
        return Rover

    if Rover.near_sample:
        # print("at sample location")
        Rover.throttle = 0
        Rover.brake = 100
        Rover.steer = 0
        return Rover
        # Rover.mode = 'stop'

    if sample_detected(Rover):
        Rover.mode = "approach_sample"

    elif path_is_blocked(Rover):
        Rover.mode = "stop"

    elif (not Rover.mode == "stop" or abs(Rover.steer) > 2) and is_stuck(Rover):
        Rover.mode = "reverse"
        Rover.last_update_time = time.time()

    # elif Rover.mode not in ("locate_sample", "stop", "reverse"):
    #     if near_sample(Rover):
    #         print("found sample")
    #         Rover.mode = "locate_sample"

    action = {
        "forward": forward_v1,
        "stop": stop_v1,
        # "locate_sample": locate_sample,
        "reverse": reverse,
        "approach_sample": approach_sample,
    }[Rover.mode]
    Rover = action(Rover)

    return Rover


def reverse(Rover):
    Rover.brake = 0
    Rover.throttle = -1

    if time.time() - Rover.last_update_time > 2:
        Rover.last_update_time = time.time()
        Rover.mode = "forward"
        Rover.throttle = Rover.throttle_set

    return Rover


def sample_detected(Rover):
    if Rover.rock_angles.size > 0:
        return True
    return False


def approach_sample(Rover):
    if Rover.rock_angles.size == 0:
        # lost visual contact with sample
        Rover.mode = "stop"
        return Rover

    angles_in_deg = np.rad2deg(Rover.rock_angles)
    rock_direction = np.mean(angles_in_deg)

    # check if facing in the right direction
    if -5 < rock_direction < 5:
        # facing in the right direction, approach
        Rover.brake = 0
        Rover.throttle = Rover.throttle_set

    else:
        # stop and turn
        if Rover.vel > 0.2:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
        else:
            Rover.brake = 0
            Rover.throttle = 0

    Rover.steer = np.clip(rock_direction, -15, 15)

    return Rover


def path_is_blocked(Rover):
    angles_in_deg = np.rad2deg(Rover.obstacles_angles)
    relevant_angles = angles_in_deg >= -15 & angles_in_deg <= 15
    relevant_dist = Rover.obstacles_dists[relevant_angles]

    if relevant_dist.size > 0:
        closest_obstacle_dist = np.min(relevant_dist)
        if closest_obstacle_dist < Rover.closest_obstacle_dist_thresh:
            return True

    return False


def is_stuck(Rover):
    if time.time() - Rover.last_update_time > 7:
        if len(Rover.history) > 10:
            xmean = 0.0
            ymean = 0.0
            for x, y in Rover.history[-3:]:
                xmean += x
                ymean += y
            xmean /= len(Rover.history[-3:])
            ymean /= len(Rover.history[-3:])
            xpos = xmean - Rover.pos[0]
            ypos = ymean - Rover.pos[1]
            dist = np.sqrt(xpos ** 2 + ypos ** 2)

            # not moving
            if dist < 0.05:
                return True
    return False


def determine_steer(Rover, direction):
    angle = direction - Rover.yaw
    if angle > 180:
        angle = - 360 + angle
    elif angle < -180:
        angle = 360 + angle

    return angle


def find_nearest_sample(Rover):
    xpos = Rover.sample_locations_to_find[0] - Rover.pos[0]
    ypos = Rover.sample_locations_to_find[1] - Rover.pos[1]

    dist = np.sqrt(xpos ** 2 + ypos ** 2)
    angles = np.rad2deg(np.arctan2(ypos, xpos))
    idx = np.argmin(dist)
    return dist[idx], angles[idx], idx


def locate_sample(Rover):
    dist, sample_direction, idx = find_nearest_sample(Rover)
    # turn until direction matches
    steer = determine_steer(Rover, sample_direction)
    if abs(steer) > 5:
        # break if still moving
        if Rover.vel > 0.1:
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            return Rover

        # print(steer)
        Rover.brake = 0
        Rover.steer = np.clip(steer, -15, 15)
        return Rover

    # direction is ok
    # print("ok")
    Rover.brake = 0
    Rover.steer = 0
    Rover.throttle = Rover.throttle_set
    return Rover


def near_sample(Rover):
    if Rover.samples_pos is None:
        return

    dist, sample_direction, idx = find_nearest_sample(Rover)
    if dist < 10:
        return True
    else:
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
            angles_in_deg = np.rad2deg(Rover.nav_angles)
            if np.sum(angles_in_deg >= 0) > np.sum(angles_in_deg < 0):
                Rover.steer = 15
            else:
                Rover.steer = -15

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
