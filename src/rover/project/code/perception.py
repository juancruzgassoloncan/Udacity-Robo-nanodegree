import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only


def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
        & (img[:, :, 1] > rgb_thresh[1]) \
        & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def color_thresh_hsv(img, low_thresh=(20, 85, 85), high_thresh=(35, 255, 255), inv=False, roi=None):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, low_thresh, high_thresh, dst=None)
    if roi is not None:
        mask = roi * mask
    if inv is True:
        mask = -(mask - 1)
    # mask = cv2.blur(mask, (7, 7))
    res = cv2.bitwise_and(img, img, dst=None, mask=mask)
    return res, mask


def segmentation(img, mask):
    res = cv2.bitwise_and(img, img, dst=None, mask=mask)
    return res


def world_segmentation(img,
                       l_r_thresh=(20, 65, 65),
                       h_r_thresh=(30, 255, 255),
                       l_n_thresh=(0, 0, 160),
                       h_n_thresh=(255, 120, 255),
                       roi=None,
                       l_o_thresh=(0, 40, 0),
                       h_o_thresh=(255, 255, 120)):
    res_r, mask_r = color_thresh_hsv(img, l_r_thresh, h_r_thresh)
    res_n, mask_n = color_thresh_hsv(img, l_n_thresh, h_n_thresh, roi=roi)
    res_o, mask_o = color_thresh_hsv(img, l_o_thresh, h_o_thresh)
    return mask_n, mask_o, mask_r


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
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
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
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    return warped


def range_view(xpix, ypix, max_r=50, min_r=0):
    dist = np.sqrt(xpix**2 + ypix**2)
    rxpix = xpix[(min_r <= dist) & (dist <= max_r)]
    rypix = ypix[(min_r <= dist) & (dist <= max_r)]
    return rxpix, rypix


def rad2deg(rad):
    return (rad * 180.0) / np.pi


def deg2rad(deg):
    return (deg * np.pi) / 180.0


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    dst_size = 5
    bottom_offset = 6
    roi = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    roi[70:, :] = 1
    src = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    dst = np.float32([[image.shape[1] / 2 - dst_size,
                       image.shape[0] - bottom_offset],
                      [image.shape[1] / 2 + dst_size,
                       image.shape[0] - bottom_offset],
                      [image.shape[1] / 2 + dst_size,
                       image.shape[0] - 2 * dst_size - bottom_offset],
                      [image.shape[1] / 2 - dst_size,
                       image.shape[0] - 2 * dst_size - bottom_offset],
                      ])
    scale = 10
    world_size = Rover.ground_truth.shape[0]

    # 2) Apply perspective transform
    # p_img = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    mask_t, mask_o, mask_r = world_segmentation(image, roi=roi)
    w_mask_t = perspect_transform(mask_t, src, dst)
    w_mask_r = perspect_transform(mask_r, src, dst)
    w_mask_o = perspect_transform(mask_o, src, dst)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = mask_o
    Rover.vision_image[:, :, 1] = mask_r
    Rover.vision_image[:, :, 2] = mask_t
    Rover.vision_image = perspect_transform(Rover.vision_image, src, dst)
    # 5) Convert map image pixel values to rover-centric coords
    t_xpix, t_ypix = rover_coords(w_mask_t)
    o_xpix, o_ypix = rover_coords(w_mask_o)
    r_xpix, r_ypix = rover_coords(w_mask_r)
    vrang = 50
    rt_xpix, rt_ypix = range_view(t_xpix, t_ypix, max_r=vrang)
    ro_xpix, ro_ypix = range_view(o_xpix, o_ypix, max_r=vrang)
    rr_xpix, rr_ypix = range_view(r_xpix, r_ypix, max_r=vrang)

    # 6) Convert rover-centric pixel values to world coordinates
    wrd_t_xpix, wrd_t_ypix = pix_to_world(rt_xpix,
                                          rt_ypix,
                                          Rover.pos[0],
                                          Rover.pos[1],
                                          Rover.yaw,
                                          world_size,
                                          scale)
    wrd_o_xpix, wrd_o_ypix = pix_to_world(ro_xpix,
                                          ro_ypix,
                                          Rover.pos[0],
                                          Rover.pos[1],
                                          Rover.yaw,
                                          world_size,
                                          scale)
    wrd_r_xpix, wrd_r_ypix = pix_to_world(r_xpix, r_ypix,
                                          Rover.pos[0], Rover.pos[1],
                                          Rover.yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # print Rover.pitch, Rover.roll

    if (359.5 < Rover.pitch or Rover.pitch < 0.5) and \
            (359.5 < Rover.roll or Rover.roll < 0.5):
        # print('update map')
        Rover.worldmap[wrd_o_ypix, wrd_o_xpix, 0] += 10
        Rover.worldmap[wrd_r_ypix, wrd_r_xpix, 1] += 10
        Rover.worldmap[wrd_t_ypix, wrd_t_xpix, 2] += 10

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
    rx_t, ry_t = range_view(t_xpix, t_ypix, max_r=40, min_r=0)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(rx_t, ry_t)

    rx_o, ry_o = range_view(o_xpix, o_ypix, max_r=50, min_r=0)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(rx_o, ry_o)

    if w_mask_r.any():
        Rover.rock_dists, Rover.rock_angles = to_polar_coords(r_xpix, r_ypix)
    else:
        Rover.rock_dists = []
        Rover.rock_angles = []
    return Rover
