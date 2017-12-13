# -*- coding: utf-8 -*-
"""
Created on Sun Dec 10 00:32:09 2017

Udacity-Robotic Nanodegree
@author: Juan Cruz Gassó Loncan
"""
from pcl_helper import *
import pcl

def statical_filter(cloud, k_neighbors=50, scale_f=1.0):
    # Filter object:
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(k_neighbors)

    # Set threshold scale factor
    x = scale_f
    # mean distance + x * std_dev will be considered outlier
    # Any point with a mean distance larger than global
    outlier_filter.set_std_dev_mul_thresh(x)

    # Call the filter
    return outlier_filter.filter()


def voxel_grid_filter(cloud,l_size=0.01):
    # Voxel Grid filter
    # Create a VoxelGrid filter objet for our imput point cloud
    vox = cloud.make_voxel_grid_filter()

    # Choose a vocel (also known as leaf)
    LEAF_SIZE = l_size

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    return vox.filter()


def passThrough_filter(cloud, axis='z', axis_min=0.77, axis_max=1.1):

    # Create a PassThrough filter object.
    passThrough = cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    passThrough.set_filter_field_name(axis)
    passThrough.set_filter_limits(axis_min,axis_max)

    # Finally use the filter
    return passThrough.filter()


def plane_segmentation(cloud, max_dist=0.01):
    plc_seg = cloud.make_segmenter()
    # Set the model you wish to fit
    plc_seg.set_model_type(pcl.SACMODEL_PLANE)
    plc_seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    max_distance = max_dist
    plc_seg.set_distance_threshold(max_distance)

    return plc_seg.segment()


def get_white_cloud(cloud):
    return XYZRGB_to_XYZ(cloud)

def clustering_eu_dist(white_cloud,tol=0.05, minCsize=100,maxCsize=1500):
    # Create a cluster extraction object
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(tol)
    ec.set_MinClusterSize(minCsize)
    ec.set_MaxClusterSize(maxCsize)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    return cluster_indices

def cluster_color_assignment(white_cloud,cluster_indices):
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud