#!/usr/bin/env python

# Import modules
from pcl_helper import *
import rospy
import pcl
import rasnac_c

# TODO: Define functions as required
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


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    pcl_filt = voxel_grid_filter(pcl_data)
    # TODO: PassThrough Filter
    pcl_filt = passThrough_filter(pcl_filt)
#
#    # TODO: RANSAC Plane Segmentation

#    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = plane_segmentation(pcl_filt)
#    # TODO: Extract inliers and outliers
    cloud_table = pcl_filt.extract(inliers, negative=False)
    cloud_objects = pcl_filt.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    # Apply function to convert XYZRGB to XYZ
    white_cloud = get_white_cloud(cloud_objects)
    cluster_idx = clustering_eu_dist(white_cloud)
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    mask_cloud = cluster_color_assignment(white_cloud,cluster_idx)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_tab = pcl_to_ros(cloud_table)
    ros_cloud_obj = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(mask_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_obj)
    pcl_table_pub.publish(ros_cloud_tab)
    pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud",
                               pc2.PointCloud2,
                               pcl_callback,
                               queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects",
                                     PointCloud2,
                                     queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table",
                                    PointCloud2,
                                    queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster",
                                      PointCloud2,
                                      queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()