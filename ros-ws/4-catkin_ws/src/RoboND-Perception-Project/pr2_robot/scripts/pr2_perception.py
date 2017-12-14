#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

import time as tm
from my_helper import *
import argparse

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering
    pcl_clean = statical_filter(pcl_data, k_neighbors=10, scale_f=0.05)
    # TODO: Voxel Grid Downsampling
    pcl_filt = voxel_grid_filter(pcl_clean,l_size=0.01)
    # TODO: PassThrough Filter
    pcl_filt = passThrough_filter(pcl_filt,axis='z',axis_min=0.57,axis_max=1.1)
    pcl_filt = passThrough_filter(pcl_filt,axis='y',axis_min=-0.46,axis_max=0.46)
    pcl_filt = passThrough_filter(pcl_filt,axis='x',axis_min=0.4,axis_max=1)
    # TODO: RANSAC Plane Segmentation
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = plane_segmentation(pcl_filt)
    # TODO: Extract inliers and outliers
    cloud_table = pcl_filt.extract(inliers, negative=False)
    cloud_objects = pcl_filt.extract(inliers, negative=True)
    # TODO: Euclidean Clustering
    # Apply function to convert XYZRGB to XYZ
    white_cloud = get_white_cloud(cloud_objects)
    cluster_idx = clustering_eu_dist(white_cloud,tol=0.05,minCsize=50,maxCsize=8000)
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    mask_cloud = cluster_color_assignment(white_cloud,cluster_idx)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_tab = pcl_to_ros(cloud_table)
    ros_cloud_obj = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(mask_cloud)
    ros_clean_cloud = pcl_to_ros(pcl_clean)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_obj)
    pcl_table_pub.publish(ros_cloud_tab)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    pcl_clean_pub.publish(ros_clean_cloud)
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_idx):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        c_hists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        n_hists = compute_normal_histograms(normals)
        feature = np.concatenate((c_hists, n_hists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    global robust
    global prev
    global stable
    global max_iter
#    tm.sleep(5)
    if not robust:
#        print max_iter
        max_iter -=1
        ndetect = detected_objects_labels
        if prev == detected_objects_labels:
#            print 'stable: ', stable
            stable += 1
            prev = ndetect
        else:
#            print prev
            prev = ndetect
            stable = 0
        if stable == 3:
#            print 'Stable!!'
            robust = True
        else:
            if max_iter == 0:
                robust = True
    else:
        robust = False
        prev = None
        stable = 0
        max_iter = 10
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass



# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    pick_pose = Pose()
    place_pose = Pose()
    obj_name = String()
    arm = String()
    test_scene_n = Int32()
    test_scene_n.data = args.number
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    yaml_dic = []
    for i in range(len(object_list_param)):
        # TODO: Parse parameters into individual variables
        obj_list_name = object_list_param[i]['name']
        obj_list_group = object_list_param[i]['group']
        for obj in object_list:
            if obj.label == obj_list_name:
                obj_name.data = obj_list_name
        # TODO: Get the PointCloud for a given object and obtain it's centroid
                pcl_array = ros_to_pcl(obj.cloud).to_array()
                centroid = np.mean(pcl_array, axis=0)[:3]
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])
        # TODO: Assign the arm to be used for pick_place
                if obj_list_group == 'green':
                    arm.data = 'right'
                    place = dropbox_param[1]['position']
                else:
                    arm.data = 'left'
                    place = dropbox_param[0]['position']
        # TODO: Create 'place_pose' for the object
                place_pose.position.x = place[0]
                place_pose.position.y = place[1]
                place_pose.position.z = place[2]
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dic = make_yaml_dict(test_scene_n, arm, obj_name, pick_pose, place_pose)
        yaml_dic.append(dic)
        # Wait for 'pick_place_routine' service to come up
    # TODO: Output your request parameters into output yaml file
    filename = 'output_'+str(test_scene_n.data)+'.yaml'
    send_to_yaml(filename, yaml_dic)

    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # TODO: Insert your message variables to be sent as a service request
#        resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
        resp = pick_place_routine(test_scene_n,obj_name,arm,pick_pose,place_pose)

        print("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == '__main__':

    # Globals variables
    robust = False
    prev = None
    stable = 0
    max_iter = 20


    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--number",default=1,type=int)
    args = parser.parse_args()
    # TODO: ROS node initialization
    rospy.init_node('obj_detection',anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points",
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
    pcl_clean_pub = rospy.Publisher("/pcl_clean",
                                    PointCloud2,
                                    queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers",
                                         Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects",
                                       DetectedObjectsArray,
                                       queue_size=1)
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
