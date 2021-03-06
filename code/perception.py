import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), invert=False):
    # Create an array of zeros same xy size as img, but single channel
    # dtype is bool for binary image
    color_select = np.zeros_like(img[:, :, 0], dtype=np.bool)
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    if invert:
        color_select = ~color_select
    # Return the binary image
    return color_select


def color_thresh_rock(rock_img):
    # Create an array of zeros same xy size as img, but single channel
    # dtype is bool for binary image
    color_select = np.zeros_like(rock_img[:, :, 0], dtype=np.bool)

    hsv_lower = (92, 82, 126)
    hsv_upper = (104, 255, 218)

    # convert color space to HSV
    img = cv2.cvtColor(rock_img, cv2.COLOR_BGR2HSV)

    lower_rock = np.array([92, 82, 126])
    upper_rock = np.array([104, 255, 218])

    # Threshold the HSV image to get only rock-like colors
    mask = cv2.inRange(img, lower_rock, upper_rock)
    mask = np.bool_(mask)

    # Index the array of zeros with the boolean array and set to 1
    color_select[mask] = 1
    # Return the binary image
    return color_select


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    # get a mask of the camera field of view
    # we don't want to map something as an obstacle, just because we cant see it on the camera image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask


def close_to_zero_deg(angle, max_error_deg):
    return angle < max_error_deg or angle > (360.0 - max_error_deg)


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    dst_size = 5
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    xpos, ypos = Rover.pos

    # 1) Define source and destination points for perspective transform
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    rock = color_thresh_rock(warped)
    navigable = color_thresh(warped)
    obstacles = ~navigable * mask

    # 4) Convert thresholded image pixel values to rover-centric coords
    xpix_nav, ypix_nav = rover_coords(navigable)

    # 5) Convert rover-centric pixel values to world coords
    x_pix_world, y_pix_world = pix_to_world(xpix_nav, ypix_nav, xpos, ypos, Rover.yaw, world_size, scale)

    # do the same for obstacles
    xpix_obs, ypix_obs = rover_coords(obstacles)
    x_obs_world, y_obs_world = pix_to_world(xpix_obs, ypix_obs, xpos, ypos, Rover.yaw, world_size, scale)

    xpix_rock, ypix_rock = rover_coords(rock)
    x_rock_world, y_rock_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, Rover.yaw, world_size, scale)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles

    # only consider the closest pixel, otherwise the rock gets streched out into a line
    rock_dist, rock_angle = to_polar_coords(xpix_rock, ypix_rock)
    Rover.rock_dists = rock_dist
    Rover.rock_angles = rock_angle

    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xpix_nav, ypix_nav)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles

    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xpix_obs, ypix_obs)
    Rover.obstacles_dists = rover_centric_pixel_distances
    Rover.obstacles_angles = rover_centric_angles

    # 6) Update worldmap (to be displayed on right side of screen)
    if close_to_zero_deg(Rover.pitch, 1.3) and close_to_zero_deg(Rover.roll, 1):
        Rover.worldmap[y_pix_world, x_pix_world, 2] += 10
        Rover.worldmap[y_obs_world, x_obs_world, 0] += 1
        if rock.any():
            rock_idx = np.argmin(rock_dist)
            rock_xcen = x_rock_world[rock_idx]
            rock_ycen = y_rock_world[rock_idx]
            Rover.worldmap[rock_ycen, rock_xcen, 1] = 255

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    # set color channel to the max
    Rover.vision_image[:, :, 2] = navigable.astype(np.uint8) * 255
    Rover.vision_image[:, :, 0] = obstacles.astype(np.uint8) * 255
    Rover.vision_image[:, :, 1] = rock.astype(np.uint8) * 255

    return Rover
