
# coding: utf-8

# ## Rover Project Test Notebook
# This notebook contains the functions from the lesson and provides the scaffolding you need to test out your mapping methods.  The steps you need to complete in this notebook for the project are the following:
#
# * First just run each of the cells in the notebook, examine the code and the results of each.
#
# **Note: For the online lab, data has been collected and provided for you. If you would like to try locally please do so! Please continue instructions from the continue point.**
# * Run the simulator in "Training Mode" and record some data. Note: the simulator may crash if you try to record a large (longer than a few minutes) dataset, but you don't need a ton of data, just some example images to work with.
# * Change the data directory path (2 cells below) to be the directory where you saved data
# * Test out the functions provided on your data
#
# **Continue Point**
# * Write new functions (or modify existing ones) to report and map out detections of obstacles and rock samples (yellow rocks)
# * Populate the `process_image()` function with the appropriate steps/functions to go from a raw image to a worldmap.
# * Run the cell that calls `process_image()` using `moviepy` functions to create video output
# * Once you have mapping working, move on to modifying `perception.py` and `decision.py` in the project to allow your rover to navigate and map in autonomous mode!
#
# **Note: If, at any point, you encounter frozen display windows or other confounding issues, you can always start again with a clean slate by going to the "Kernel" menu above and selecting "Restart & Clear Output".**
#
# **Run the next cell to get code highlighting in the markdown cells.**

# In[1]:


# get_ipython().run_cell_magic(u'HTML', u'', u'<style> code {background-color : orange !important;} </style>')


# In[2]:


get_ipython().magic(u'matplotlib inline')
# matplotlib qt # Choose %matplotlib qt to plot to an interactive window (note it may show up behind your browser)
# Make some of the relevant imports
import cv2  # OpenCV for perspective transform
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import scipy.misc  # For saving images as needed
import glob  # For reading in a list of images from a folder


# ## Quick Look at the Data
# There's some example data provided in the `test_dataset` folder.  This basic dataset is enough to get you up and running but if you want to hone your methods more carefully you should record some data of your own to sample various scenarios in the simulator.
#
# Next, read in and display a random image from the `test_dataset` folder

# In[167]:


# path = '../test_dataset/IMG/*'
path = '../../data/IMG/*'
img_list = glob.glob(path)
# Grab a random image and display it
idx = np.random.randint(0, len(img_list) - 1)
image = mpimg.imread(img_list[idx])
plt.imshow(image)


#
# ## Calibration Data
# Read in and display example grid and rock sample calibration images.  You'll use the grid for perspective transform and the rock image for creating a new color selection that identifies these samples of interest.

# In[168]:


# In the simulator you can toggle on a grid on the ground for calibration
# You can also toggle on the rock samples with the 0 (zero) key.
# Here's an example of the grid and one of the rocks
example_grid = '../../data/calibration_images/example_grid1.jpg'
example_rock = '../../data/calibration_images/example_rock1.jpg'
grid_img = mpimg.imread(example_grid)
rock_img = mpimg.imread(example_rock)

fig = plt.figure(figsize=(12, 3))
plt.subplot(121)
plt.imshow(grid_img)
plt.subplot(122)
plt.imshow(rock_img)


# ## Perspective Transform
#
# Define the perspective transform function from the lesson and test it on an image.

# In[169]:


# Define a function to perform a perspective transform
# I've used the example grid image above to choose source points for the
# grid cell in front of the rover (each grid cell is 1 square meter in the sim)
# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    return warped


# Define calibration box in source (actual) and destination (desired) coordinates
# These source and destination points are defined to warp the image
# to a grid where each 10x10 pixel square represents 1 square meter
# The destination box will be 2*dst_size on each side
dst_size = 5
# Set a bottom offset to account for the fact that the bottom of the image
# is not the position of the rover but a bit in front of it
# this is just a rough guess, feel free to change it!
bottom_offset = 6
source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                          [image.shape[1] / 2 + dst_size,
                              image.shape[0] - bottom_offset],
                          [image.shape[1] / 2 + dst_size, image.shape[0] -
                              2 * dst_size - bottom_offset],
                          [image.shape[1] / 2 - dst_size, image.shape[0] -
                              2 * dst_size - bottom_offset],
                          ])
warped = perspect_transform(grid_img, source, destination)
plt.imshow(warped)
#scipy.misc.imsave('../output/warped_example.jpg', warped)


# ## Color Thresholding
# Define the color thresholding function from the lesson and apply it to the warped image
#
# **TODO:** Ultimately, you want your map to not just include navigable terrain but also obstacles and the positions of the rock samples you're searching for.  Modify this function or write a new function that returns the pixel locations of obstacles (areas below the threshold) and rock samples (yellow rocks in calibration images), such that you can map these areas into world coordinates as well.
# **Suggestion:** Think about imposing a lower and upper boundary in your color selection to be more specific about choosing colors.  Feel free to get creative and even bring in functions from other libraries.  Here's an example of [color selection](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html) using OpenCV.
# **Beware:** if you start manipulating images with OpenCV, keep in mind that it defaults to `BGR` instead of `RGB` color space when reading/writing images, so things can get confusing.

# In[170]:


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) & (
        img[:, :, 1] > rgb_thresh[1]) & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


threshed = color_thresh(warped)
plt.imshow(threshed, cmap='gray')
#scipy.misc.imsave('../output/warped_threshed.jpg', threshed*255)


# In[28]:


import cv2
import numpy as np
import glob
# cap = cv2.VideoCapture(0)


def nothing(x):
    pass


# Creamos una ventana llamada 'image' en la que habra todos los sliders
cv2.namedWindow('image')
cv2.createTrackbar('Hue Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Hue Max', 'image', 0, 255, nothing)
cv2.createTrackbar('Sat Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Sat Max', 'image', 0, 255, nothing)
cv2.createTrackbar('Val Min', 'image', 0, 255, nothing)
cv2.createTrackbar('Val Max', 'image', 0, 255, nothing)
# set initial values
cv2.setTrackbarPos('Hue Min', 'image', 0)
cv2.setTrackbarPos('Hue Max', 'image', 255)
cv2.setTrackbarPos('Sat Min', 'image', 0)
cv2.setTrackbarPos('Sat Max', 'image', 70)
cv2.setTrackbarPos('Val Min', 'image', 70)
cv2.setTrackbarPos('Val Max', 'image', 255)

path = '../test_dataset/IMG/*'
img_list = glob.glob(path)
# Grab a random image and display it
idx = np.random.randint(0, len(img_list) - 1)
frame = cv2.imread(img_list[idx])


while(1):
    # idx = np.random.randint(0, len(img_list) - 1)
    # frame = cv2.imread(img_list[idx])
    # _,frame = cap.read() #Leer un frame
    # Convertirlo a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Los valores maximo y minimo de H,S y V se guardan en funcion de la posicion de los sliders
    hMin = cv2.getTrackbarPos('Hue Min', 'image')
    hMax = cv2.getTrackbarPos('Hue Max', 'image')
    sMin = cv2.getTrackbarPos('Sat Min', 'image')
    sMax = cv2.getTrackbarPos('Sat Max', 'image')
    vMin = cv2.getTrackbarPos('Val Min', 'image')
    vMax = cv2.getTrackbarPos('Val Max', 'image')

    # Se crea un array con las posiciones minimas y maximas
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Deteccion de colores
    mask = cv2.inRange(hsv, lower, upper)
#     roi = np.zeros_like(mask)
    roi = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    roi[70:, :] = 1

    mask_roi = mask * roi
    threshed = cv2.bitwise_and(frame, frame, dst=None, mask=mask_roi)

    # Mostrar los resultados y salir
    cv2.imshow('camara', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('thresed', threshed)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    elif k == 97:
        idx = np.random.randint(0, len(img_list) - 1)
        frame = cv2.imread(img_list[idx])

cv2.destroyAllWindows()


# In[475]:


def color_thresh_hsv(img, low_thresh=(20, 85, 85), high_thresh=(35, 255, 255), inv=False, roi=None):
    """Color thresholding in HSV color space.

    Arguments
    ---------
    img: numpy.ndarray
        rgb image array
    low_thresh: tuple
        Lower boundary color in (h,s,v)
    high_thresh: tuple
        Higher boundary color in (h,s,v)
    roi: numpy.ndarray
        binary mask for region of interest.
    Returns
    -------
    res: numpy.ndarray
        The resulting confusion threshed image.
    mask: numpy.ndarray
        The segmentation mask.
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, low_thresh, high_thresh, dst=None) / 255

    if roi is not None:
        mask = roi * mask
    if inv is True:
        mask = -(mask - 1)
    mask = cv2.blur(mask, (7, 7))
    res = cv2.bitwise_and(img, img, dst=None, mask=mask)
    return res, mask


# In[476]:


rgb_c = np.array([[[160, 160, 160]]]).astype('uint8')
hsv_c = cv2.cvtColor(rgb_c, cv2.COLOR_RGB2HSV)
print hsv_c


# In[477]:


low_n_thresh = (0, 0, 180)
high_n_thresh = (255, 80, 255)
roi = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
roi[70:, :] = 1
threshed, mask = color_thresh_hsv(image, low_n_thresh, high_n_thresh, roi=roi)
plt.imshow(mask, cmap='gray')


# In[486]:


def world_segmentation(img,
                       l_r_thresh=(20, 65, 65),
                       h_r_thresh=(35, 255, 255),
                       l_n_thresh=(0, 0, 120),
                       h_n_thresh=(255, 80, 255),
                       roi=None,
                       l_o_thresh=(0, 0, 0),
                       h_o_thresh=(255, 255, 120)):
    res_r, mask_r = color_thresh_hsv(img, l_r_thresh, h_r_thresh)
    res, mask_n = color_thresh_hsv(img, l_n_thresh, h_n_thresh, roi=roi)
    res, mask_o = color_thresh_hsv(
        img, l_n_thresh, h_n_thresh, roi=roi, inv=True)
    return mask_n, mask_o, mask_r
    #     res, mask_o = color_thresh_hsv(img, l_o_thresh, h_o_thresh)


idx = np.random.randint(0, len(img_list) - 1)
image = plt.imread(img_list[idx])
roi1 = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
# roi2 = np.ones((image.shape[0],image.shape[1]),dtype=np.uint8)
roi1[70:, :] = 1

# roi2[:,:] = 0
T, O, R = world_segmentation(image, roi=roi1)
# Or = cv2.bitwise_and(O,O,roi2)
# b = cv2.blur(T, (7,7))#, dst=None, anchor=None, borderType=None)
# b[b!=255]=0
# Or=R
plt.figure(figsize=(6, 6), dpi=120)
plt.subplot(221)
plt.imshow(T, 'gray')
plt.subplot(222)
plt.imshow(O, 'gray')
plt.subplot(223)
plt.imshow(R, 'gray')
plt.subplot(224)
plt.imshow(image, 'gray')


# In[445]:
rock_img_s = image.copy()


# In[425]:


colored_warped = np.zeros_like(warped)
colored_warped[:, :, 0] = O * 170
colored_warped[:, :, 1] = R * 255
colored_warped[:, :, 2] = T * 175
plt.imshow(colored_warped)


# ## Coordinate Transformations
# Define the functions used to do coordinate transforms and apply them to an image.

# In[616]:


def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

# Define a function to apply a rotation to pixel positions


def rotate_pix(xpix, ypix, yaw):
    yaw_r = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_r) - ypix * np.sin(yaw_r)
    ypix_rotated = xpix * np.sin(yaw_r) + ypix * np.cos(yaw_r)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = xpix_rot / scale + xpos
    ypix_translated = ypix_rot / scale + ypos
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

# Grab another random image
# idx = np.random.randint(0, len(img_list)-1)


def range_view(xpix, ypix, max_r=50, min_r=0):
    dist = np.sqrt(xpix**2 + ypix**2)
    rxpix = xpix[(min_r <= dist) & (dist <= max_r)]
    rypix = ypix[(min_r <= dist) & (dist <= max_r)]
    # rxpix = xpix[]
    # rypix = ypix[min_r < dist]
    return rxpix, rypix


dist = 10, 1, 4, 50, 55


def rad2deg(rad):
    return rad * 180 / np.pi




def get_wall_distance(polar_points, arc=15):
    wall_d = [d for d, a in polar_points if (a * 180 / np.pi) < -arc]
    wall_i = [d for d, a in polar_points if (a * 180 / np.pi) > arc]
    return np.array(wall_i) if len(wall_i) else np.array(0),\
        np.array(wall_d) if len(wall_d) else np.array(0)


# %%
idx += 50
try:
    image = mpimg.imread(img_list[idx])
except IndexError:
    idx = 0
# plt.imshow(image)
# %

roi = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
roi[70:, :] = 1
threshed = color_thresh(image)
res, mask_n = color_thresh_hsv(image, (0, 0, 120), (255, 80, 255), roi=roi)
res2, mask_r = color_thresh_hsv(image, (20, 80, 80), (35, 255, 255))
# b_mask_n = cv2.blur(mask_n, (10,10))
# print b_mask_n
warped = perspect_transform(mask_n, source, destination)
warped2 = perspect_transform(mask_r, source, destination)
# threshed = color_thresh(warped)

# Calculate pixel values in rover-centric coords and distance/angle to all pixels

# navigable
xpix, ypix = rover_coords(warped)
xpix, ypix = range_view(xpix, ypix, max_r=30, min_r=2)
dist, angles = to_polar_coords(xpix, ypix)
da = np.array((dist, angles)).T

# rocks
xpix2, ypix2 = rover_coords(warped2)
xpix2, ypix2 = range_view(xpix2, ypix2, min_r=5)
dist2, angles2 = to_polar_coords(xpix2, ypix2)
da2 = np.array((dist2, angles2)).T


s_ang = 0
n = 0
near_a = np.array([a for d, a in da if d < 40])
da[0]
center_dist = np.array([d for d, a in da if -10 < rad2deg(a) < 10])
# for d,a in da:
#     if d < 20:
#         s_ang += a
#         n += 1
# mean_dir = np.mean(angles)
mean_dir = np.mean(near_a)
desv = np.sqrt(near_a.var())

# Do some plotting
s_ang = np.var(near_a)
fig = plt.figure(figsize=(12, 9))
plt.subplot(221)
plt.imshow(image)
plt.subplot(222)
plt.imshow(warped, cmap='gray')
plt.subplot(223)
plt.imshow(res, cmap='gray')
plt.subplot(224)
plt.plot(xpix, ypix, '.')
plt.plot(xpix2, ypix2, '.')

plt.ylim(-160, 160)
plt.xlim(0, 160)
arrow_length = 100
x_arrow = arrow_length * np.cos(mean_dir)
y_arrow = arrow_length * np.sin(mean_dir)
dx_arrow1 = arrow_length * np.cos(mean_dir + desv)
dy_arrow1 = arrow_length * np.sin(mean_dir + desv)
dx_arrow2 = arrow_length * np.cos(mean_dir - desv)
dy_arrow2 = arrow_length * np.sin(mean_dir - desv)
plt.arrow(0, 0, x_arrow, y_arrow, color='red',
          zorder=2, head_width=10, width=2)
plt.arrow(0, 0, dx_arrow1, dy_arrow1, color='yellow',
          zorder=2, head_width=0, width=1)
plt.arrow(0, 0, dx_arrow2, dy_arrow2, color='red',
          zorder=2, head_width=0, width=1)
tita = rad2deg(mean_dir + desv)
mean_dist = center_dist.mean()
wall_dist = get_wall_distance(da, tita)

print 'tita:', tita, ' median tita: ', rad2deg(mean_dir), ' dist_media: ', mean_dist
print 'size land: ', len(angles), 'wall i: ', wall_dist[0].min(), 'wall d: ', wall_dist[1].min()
if da2.shape[0] > 0:
    print 'piedra d:', da2[:, 0].mean(), ' a: ', rad2deg(da2[:, 1].mean())
# if warped[125:,:160].sum() < warped[125:,160:].sum():
#     if warped[125:].sum()*0.7 < warped[125:,160:].sum():
#         print 'go ahead'
# else:
#     print 'turn left'


# In[579]:


# idx=2
# warped[:,:160].sum() > warped[:,160].sum()


# ## Read in saved data and ground truth map of the world
# The next cell is all setup to read your saved data into a `pandas` dataframe.  Here you'll also read in a "ground truth" map of the world, where white pixels (pixel value = 1) represent navigable terrain.
#
# After that, we'll define a class to store telemetry data and pathnames to images.  When you instantiate this class (`data = Databucket()`) you'll have a global variable called `data` that you can refer to for telemetry and map data within the `process_image()` function in the following cell.
#

# In[352]:


# Import pandas and read in csv file as a dataframe
import pandas as pd
# Change this path to your data directory
# df = pd.read_csv('../test_dataset/robot_log.csv', delimiter=';', decimal=',')
df = pd.read_csv('../../data/robot_log.csv', delimiter=';', decimal=',')
df.head()
csv_img_list = df["Path"].tolist()  # Create list of image pathnames
# Read in ground truth map and create a 3-channel image with it
ground_truth = mpimg.imread('../../data/calibration_images/map_bw.png')
ground_truth_3d = np.dstack(
    (ground_truth * 0, ground_truth * 255, ground_truth * 0)).astype(np.float)

# Creating a class to be the data container
# Will read in saved data from csv file and populate this object
# Worldmap is instantiated as 200 x 200 grids corresponding
# to a 200m x 200m space (same size as the ground truth map: 200 x 200 pixels)
# This encompasses the full range of output position values in x and y from the sim


class Databucket():
    def __init__(self):
        self.images = csv_img_list
        self.xpos = df["X_Position"].values
        self.ypos = df["Y_Position"].values
        self.yaw = df["Yaw"].values
        self.count = -1  # This will be a running index, setting to -1 is a hack
        # because moviepy (below) seems to run one extra iteration
        self.worldmap = np.zeros((200, 200, 3)).astype(np.float)
        self.ground_truth = ground_truth_3d  # Ground truth worldmap


# Instantiate a Databucket().. this will be a global variable/object
# that you can refer to in the process_image() function below
data = Databucket()


# ## Write a function to process stored images
#
# Modify the `process_image()` function below by adding in the perception step processes (functions defined above) to perform image analysis and mapping.  The following cell is all set up to use this `process_image()` function in conjunction with the `moviepy` video processing package to create a video from the images you saved taking data in the simulator.
#
# In short, you will be passing individual images into `process_image()` and building up an image called `output_image` that will be stored as one frame of video.  You can make a mosaic of the various steps of your analysis process and add text as you like (example provided below).
#
#
#
# To start with, you can simply run the next three cells to see what happens, but then go ahead and modify them such that the output video demonstrates your mapping process.  Feel free to get creative!

# In[353]:
def range_view(xpix, ypix, rang=50):
    dist = np.sqrt(xpix ** 2 + ypix ** 2)
    return xpix[dist < rang], ypix[dist < rang]


img = image
# Define a function to pass stored images to
# reading rover position and yaw angle from csv file
# This function will be used by moviepy to create an output video


def process_image(img):
    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])

    # TODO:
    # 1) Define source and destination points for perspective transform
    src = source
    dst = destination
    scale = 10
    roi = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    roi[70:, :] = 1
    img_path = data.images[data.count]
    img = mpimg.imread(img_path)
    world_size = data.worldmap.shape[0]
    # 2) Apply perspective transform
#     p_img = perspect_transform(img, src, dst)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    mask_t, mask_o, mask_r = world_segmentation(img, roi=roi)
    w_mask_t = perspect_transform(mask_t, src, dst)
    w_mask_r = perspect_transform(mask_r, src, dst)
    w_mask_o = perspect_transform(mask_o, src, dst)

    # 4) Convert thresholded image pixel values to rover-centric coords
    t_xpix, t_ypix = rover_coords(w_mask_t)
    o_xpix, o_ypix = rover_coords(w_mask_o)
    r_xpix, r_ypix = rover_coords(w_mask_r)

    # 4-a) range of view
    t_xpix_r, t_ypix_r = range_view(t_xpix, t_ypix)
    o_xpix_r, o_ypix_r = range_view(o_xpix, o_ypix)
    r_xpix_r, r_ypix_r = range_view(r_xpix, r_ypix)
    # 5) Convert rover-centric pixel values to world coords
    xpos = np.float_(data.xpos[data.count])
    ypos = np.float_(data.ypos[data.count])
    yaw = np.float_(data.yaw[data.count])
    wrd_t_xpix, wrd_t_ypix = pix_to_world(t_xpix_r,
                                          t_ypix_r,
                                          xpos,
                                          ypos,
                                          yaw,
                                          world_size,
                                          scale)
    wrd_o_xpix, wrd_o_ypix = pix_to_world(o_xpix_r,
                                          o_ypix_r,
                                          xpos,
                                          ypos,
                                          yaw,
                                          world_size,
                                          scale)
    wrd_r_xpix, wrd_r_ypix = pix_to_world(r_xpix_r, r_ypix_r,
                                          xpos, ypos,
                                          yaw, world_size, scale)
    # 6) Update worldmap (to be displayed on right side of screen)
    # Example: data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          data.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    data.worldmap[wrd_o_ypix, wrd_o_xpix, 0] += 1
    data.worldmap[wrd_r_ypix, wrd_r_xpix, 1] += 1
    data.worldmap[wrd_t_ypix, wrd_t_xpix, 2] += 2

    # 7) Make a mosaic image, below is some example code
    # First create a blank image (can be whatever shape you like)
    output_image = np.zeros(
        (img.shape[0] + data.worldmap.shape[0], img.shape[1] * 2, 3))
    # Next you can populate regions of the image with various output
    # Here I'm putting the original image in the upper left hand corner
    output_image[0:img.shape[0], 0:img.shape[1]] = img

    # Let's create more images to add to the mosaic, first a warped image
    warped = perspect_transform(img, source, destination)
    # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped

    # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.3, 0.7)
    # Flip map overlay so y-axis points upward and add to output_image
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
#         # Colored warped img
    colored_warped = np.zeros_like(warped)
    colored_warped[:, :, 0] = w_mask_o * 170
    colored_warped[:, :, 1] = w_mask_r * 170
    colored_warped[:, :, 2] = w_mask_t * 170
    output_image[img.shape[0]:img.shape[0] +
                 warped.shape[0], img.shape[1]:] = colored_warped

    # Then putting some text over the image
    cv2.putText(output_image, "My video", (20, 20),
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    data.count += 1  # Keep track of the index in the Databucket()

    return output_image


# In[354]:


data.count = -1
out = process_image(image)


# In[355]:


image.shape
# plt.imshow(out[:160,320:,:])
# plt.imshow(data.ground_truth[:,:,1])
data.ground_truth[:, :, 1].min


# ## Make a video from processed image data
# Use the [moviepy](https://zulko.github.io/moviepy/) library to process images and create a video.
#

# In[356]:


# Import everything needed to edit/save/watch video clips
from moviepy.editor import VideoFileClip
from moviepy.editor import ImageSequenceClip


# Define pathname to save the output video
output = './output/test_mapping_mine.mp4'
data = Databucket()  # Re-initialize data in case you're running this cell multiple times
# Note: output video will be sped up because
clip = ImageSequenceClip(data.images, fps=60)
# recording rate in simulator is fps=25
# NOTE: this function expects color images!!
new_clip = clip.fl_image(process_image)
get_ipython().magic(u'time new_clip.write_videofile(output, audio=False)')


# ### This next cell should function as an inline video player
# If this fails to render the video, try running the following cell (alternative video rendering method).  You can also simply have a look at the saved mp4 in your `/output` folder

# In[104]:


output = './output/test_mapping_mine.mp4'
from IPython.display import HTML
HTML("""
<video width="960" height="540" controls>
  <source src="{0}">
</video>
""".format('./output/test_mapping_mine.mp4'))


# In[105]:


import io
import base64
video = io.open(output, 'r+b').read()
encoded_video = base64.b64encode(video)
HTML(data='''<video alt="test" controls>
                <source src="data:video/mp4;base64,{0}" type="video/mp4" />
             </video>'''.format(encoded_video.decode('ascii')))
