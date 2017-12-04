import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as nps
import cv2
from extra_functions import perspect_transform, color_thresh, source, destination

# Read in the sample image
image_name = '../data/IMG/robocam_2017_10_03_15_35_32_475.jpg'
image = mpimg.imread(image_name)


def rover_coords(binary_img):
    # TODO: fill in this function to
    # Calculate pixel positions with reference to the rover
    # position being at the center bottom of the image.
    yb, xb = binary_img.nonzero()
    y_pixel = -(xb - binary_img.shape[0])
    x_pixel = -(yb - binary_img.shape[1] / 2)
    return x_pixel, y_pixel


# Perform warping and color thresholding
warped = perspect_transform(image, source, destination)
colorsel = color_thresh(warped, rgb_thresh=(160, 160, 160))
# Extract x and y positions of navigable terrain pixels
# and convert to rover coordinates
xpix, ypix = rover_coords(colorsel)

# Plot the map in rover-centric coords
fig = plt.figure(figsize=(5, 7.5))
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
plt.title('Rover-Centric Map', fontsize=20)
plt.show()  # Uncomment if running on your local machine
