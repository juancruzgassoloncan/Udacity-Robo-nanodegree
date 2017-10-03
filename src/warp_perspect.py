import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2


image_name = '../data/IMG/robocam_2017_10_03_15_35_32_475.jpg'
image = mpimg.imread(image_name)


def perspect_transform(img, src, dst):

    # Get transform matrix using cv2.getPerspectivTransform()
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp image using cv2.warpPerspective()
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    # Return the result
    return warped


# TODO:
# Define a box in source (original) and
# destination (desired) coordinates
# Right now source and destination are just
# set to equal the four corners
# of the image so no transform is taking place
# Try experimenting with different values!
source = np.float32([[35, 135],
                     [120, 97],
                     [202, 97],
                     [300, 135]])
# source = np.float32([[0, 0],
#                  [0, image.shape[0]],
#                  [image.shape[1], image.shape[0]],
#                  [image.shape[1], 0]])
# destination = np.float32([[0, 0],
#                  [0, image.shape[0]],
#                  [image.shape[1], image.shape[0]],
#                  [image.shape[1], 0]])
dst_size = 5
bottom_offset = 15
destination = np.float32([[image.shape[1] / 2 - dst_size,
                           image.shape[0] - bottom_offset],
                          [image.shape[1] / 2 - dst_size, 2 *
                              dst_size + image.shape[0] - bottom_offset],
                          [image.shape[1] / 2 + dst_size, 2 *
                              dst_size + image.shape[0] - bottom_offset],
                          [image.shape[1] / 2 + dst_size,
                           image.shape[0] - bottom_offset]])

warped = perspect_transform(image, source, destination)
# Draw Source and destination points on images (in blue) before plotting
cv2.polylines(image, np.int32([source]), True, (0, 0, 255), 2)
cv2.polylines(warped, np.int32([destination]), True, (0, 0, 255), 2)
# Display the original image and binary
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 6), sharey=True)
f.tight_layout()
ax1.imshow(image)
ax1.set_title('Original Image', fontsize=40)

ax2.imshow(warped, cmap='gray')
ax2.set_title('Result', fontsize=40)
plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
plt.show()  # Uncomment if running on your local machine
