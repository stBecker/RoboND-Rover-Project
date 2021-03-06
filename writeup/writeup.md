## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: color_tresh_navigable.png
[image2]: color_tresh_obstacle.png
[image3]: color_tresh_rock.png
[image4]: screenshot_sim.png
[image5]: full.png
[image6]: angle.png
[image7]: dist.png
[image8]: angle_dist.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
##### Obstacles
To detect obstacles the color threshholded image of navigable terrain is simply inverted:

    color_select[above_thresh] = 1
    if invert:
        color_select = np.invert(color_select)

Navigable:

![alt text][image1]

Obstacle (inverted Navigable):

![alt text][image2]

##### Rocks
For rock detection we need to define lower and upper bounds for color threshholds.
The bounds were estimated manually by observing the provided rock example image.

    rock_settings = (
        (130, 105, 0), # lower bound RGB
        (210, 185, 90), # upper bound RGB
    )

The code for upper bound handling was integrated with the treshholding function:

    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])

    if rgb_upper_bound:
        below_thresh = (img[:,:,0] < rgb_upper_bound[0]) \
                & (img[:,:,1] < rgb_upper_bound[1]) \
                & (img[:,:,2] < rgb_upper_bound[2])
        above_thresh = above_thresh & below_thresh

Rock:

![alt text][image3]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
First the input image is warped to a "birds-eye" perspective, to match the perspective of the world map.

    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])

    warped = perspect_transform(image, source, destination)

The warped image is color threshholded to identify navigable terrain and rocks.

    rock_settings = (
        (130, 105, 0),
        (210, 185, 90),
        )
    rock = color_thresh(warped, rgb_thresh=rock_settings[0], rgb_upper_bound=rock_settings[1])
    navigable = color_thresh(warped)
    obstacles = color_thresh(warped, invert=True)
    terrain_types = (obstacles, rock, navigable)


The color threshholded images are then transformed into a rover-centric coordinate system (i.e. the rover is at position (0, 0)),
and ultimately into world coordinates (by rotating by the rover yaw, scaling down to the world map size and translating with the rovers current position).
The world map pixels are then updated with the identified terrain types, red for obstacles, green for rocks, blue for navigable terrain.
More intense colors signify a higher degree of confidence in the mapping.


    for i, threshed in enumerate(terrain_types):
        xpix, ypix = rover_coords(threshed)
        x_pix_world, y_pix_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        data.worldmap[y_pix_world, x_pix_world, i] += 1


The output images are rendered, the world map is overlayed with the ground truth map.


    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
    output_image[0:img.shape[0], 0:img.shape[1]] = img
    warped = perspect_transform(img, source, destination)
    output_image[0:img.shape[0], img.shape[1]:] = warped
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)

A video recording for the example data can be found in the output folder.


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### perception_step
The perception_step was implemented for the most part like the process_image function described above.
Notable differences to the reference implementation:

1. The world map is only updated when pitch and roll are close (within about 1 degree) to zero. This is to prevent invalid mappings.


        def close_to_zero_deg(angle, max_error_deg):
            return angle < max_error_deg or angle > (360.0 - max_error_deg)

         if close_to_zero_deg(Rover.pitch, 1.3) and close_to_zero_deg(Rover.roll, 1):
                Rover.worldmap[y_pix_world, x_pix_world, i] += 1

2. The distances and angles of each color threshholded image, with the rover as a reference point, are recorded.

        x_pixel, y_pixel = rover_coords(navigable)
        rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(x_pixel, y_pixel)
        Rover.nav_dists = rover_centric_pixel_distances
        Rover.nav_angles = rover_centric_angles

        x_pixel, y_pixel = rover_coords(rock)
        rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(x_pixel, y_pixel)
        Rover.rock_dists = rover_centric_pixel_distances
        Rover.rock_angles = rover_centric_angles

        x_pixel, y_pixel = rover_coords(obstacles)
        rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(x_pixel, y_pixel)
        Rover.obstacles_dists = rover_centric_pixel_distances
        Rover.obstacles_angles = rover_centric_angles

##### decision_step
1. Refactored function to use a mapping instead of if .. elif .. clauses, for better readability:

        action = {
            "forward": forward_v1,
            "stop": stop_v1,
            "reverse": reverse,
            "approach_sample": approach_sample,
        }[Rover.mode]
        Rover = action(Rover)


2. Detect and approach samples

When there are sufficiently many "rock" pixels detected (the threshold is > 1 to limit spurious detections),
the rover enters the "approach_sample" mode. It steers in the general direction of the detected rock_angles and
slowly (in order not to shot by the target) approaches the sample. When visual contact with the sample is lost, 
change the mode.


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


3. Detect obstacles in the way

The idea is to detect if there is an obstacle blocking the way. This is determined by checking for obstacle pixels
within a certain area in front of the rover.

Original obstacle detection

        plt.imshow(obstacles)

        # calculate distances to rover
        m = np.zeros_like(obs2, dtype=np.float)
        for i in range(m.shape[0]):
            for j in range(m.shape[1]):
                m[i][j] = np.sqrt((i - 160.0)**2 + (j-160.0)**2)

        # calculate angles to rover
        n = np.zeros_like(obs2, dtype=np.float)
        for i in range(n.shape[0]):
            for j in range(n.shape[1]):
                n[i][j] = np.rad2deg(np.arctan2(j - 160.0, i - 160.0))

![alt text][image5]

Area within a certain angle

        plt.imshow(obs2 & ((n < -160) | (n > 160)))

![alt text][image6]

Area within a certain radius

        plt.imshow(obs2 & (m > 7) & (m < 40))

![alt text][image7]

The combination is relevant for navigation

        plt.imshow(obs2 & (m > 7) & (m < 70) & ((n < -160) | (n > 160)))

![alt text][image8]


The implementation is given below. If there are enough obstacle pixels within the relevant area, assume the path to be blocked.
"Relevant" is defined as having an angle between -10 and 10 degrees and being at least 8 pixels (the area directly in front of the rover
is always detected as an obstacle, because we have no vision data for this area) but less than 30 pixels away from
the rover.

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


4. Determine if rover is stuck and back up to become unstuck

If the rover has not moved significantly within the last several seconds, it is probably stuck.
In this case we put the rover in reverse for a few seconds.


        def is_stuck(Rover):
            if time.time() - Rover.last_update_time > 15:
                # how far have we moved within the last x seconds
                xpos = Rover.last_pos[0] - Rover.pos[0]
                ypos = Rover.last_pos[1] - Rover.pos[1]
                dist = np.sqrt(xpos ** 2 + ypos ** 2)

                # update values
                Rover.last_update_time = time.time()
                Rover.last_pos = Rover.pos

                print("dist to POS: ",dist)
                if dist < 15:
                    return True

            return False

        def reverse(Rover):
            Rover.brake = 0
            Rover.throttle = -1

            if time.time() - Rover.last_update_time > 2:
                Rover.last_update_time = time.time()
                Rover.mode = "forward"
                Rover.throttle = Rover.throttle_set

            return Rover

5. Smooth steering

The Rover.steer value might change drastically (e.g. if there is a low number of nav_angles), which results in unstable navigation.
By smoothing the steering angle, the driving behaviour is improved.

        steer_diff = abs(Rover.steer - Rover.previous_steer)
        if steer_diff > Rover.steer_set:
            # smooth out steering angle
            if Rover.steer >= Rover.previous_steer:
                # turning left
                Rover.steer = Rover.previous_steer + Rover.steer_set
            else:
                Rover.steer = Rover.previous_steer - Rover.steer_set

        Rover.previous_steer = Rover.steer


6. Turning when stopped

Changed the default steer of -15 to a more clever solution:
When there is more navigation data on the right, then turn to the right, else turn to the left.
When we have committed to one direction, we keep going that way no matter what. This is necessary to prevent the rover from
oscillating between left and right, e.g. if it has turned a bit to the left and suddenly there is more navigation data to the right.

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


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

###### Specs
- 1280 x 1024
- Quality: Fantastic
- FPS: 40 - 50

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

![alt text][image4]

###### Failed approaches
1. Clever turning when stopped:

        # Rover.steer = -15  # Could be more clever here about which way to turn
        angles_in_deg = np.rad2deg(Rover.nav_angles)
        if np.sum(angles_in_deg >= 0) > np.sum(angles_in_deg < 0):
            Rover.steer = 15
        else:
            Rover.steer = -15

Sometimes the rover would get stuck in front of an obstacle, with about an equal number of nav_angles in both directions.
It would the oscillate between turning left and right, while the direction with more nav_angles would constantly flip.

2. Locate samples by checking defined positions:

        if Rover.sample_locations_to_find is None:
            Rover.sample_locations_to_find = np.copy(Rover.samples_pos)

        def find_nearest_sample(Rover):
            xpos = Rover.sample_locations_to_find[0] - Rover.pos[0]
            ypos = Rover.sample_locations_to_find[1] - Rover.pos[1]

            dist = np.sqrt(xpos ** 2 + ypos ** 2)
            angles = np.rad2deg(np.arctan2(ypos, xpos))
            idx = np.argmin(dist)
            return dist[idx], angles[idx], idx

         def near_sample(Rover):
            if Rover.samples_pos is None:
                return

            dist, sample_direction, idx = find_nearest_sample(Rover)
            if dist < 10:
                return True
            else:
                return False

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


Use the sample locations stored in "Rover.sample_pos" to locate and approach samples.
Worked reasonably well, but was considered to be "cheating".


3. Wall crawler strategy:

Find the furthest nav_angle to the right and determine if it's not that far to the right after all.

        # near wall
        if angles_in_degree.size > 0:
            # obstacle to the right
            if -5 < np.min(angles_in_degree) < -1:
                Rover.mode = "wall_right"
                print("wall_right")
                return Rover

When in "wall_right" mode:
Try to maintain the current ratio of non_navigable terrain on the right hand side, initiating course corrections if necessary.

        elif Rover.mode == "wall_right":
            angles_in_degree = Rover.nav_angles * 180 / np.pi
            Rover.throttle = Rover.throttle_set
            Rover.steer = 0
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

Should work in principle, but was ultimately abandoned for simpler strategies. The above implementation could follow a straight wall reasonably well, but would fail miserably when the wall curved or a rock obstacle appeared.


##### Improvements
1. Return to starting location when all sample collected:
Could add a waypoint or simply drive around randomly until reasonably close.
2. "Wall crawler" strategy:
Try to always have some obstacle on the right/left of the rover.
3. Remember previously visited locations:
Keep a list of visited coordinates and penalize driving towards them.

