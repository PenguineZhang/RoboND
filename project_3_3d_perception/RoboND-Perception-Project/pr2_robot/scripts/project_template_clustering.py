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


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
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
    
    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter() 
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    
    pcl_data_vox_filter = vox.filter()
    
    # TODO: PassThrough Filter
    vox_pt_filter = pcl_data_vox_filter.make_passthrough_filter()
    filter_axis = 'z'
    vox_pt_filter.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    vox_pt_filter.set_filter_limits(axis_min, axis_max)
    pcl_pt_filtered = vox_pt_filter.filter()

    # TODO: Remove noise
    # start by creating a filter object
    outlier_filter = pcl_pt_filtered.make_statistical_outlier_filter()
    
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(15)

    # Set threshold scale factor
    x = 0.01

    # Any point with a mean distance larger than global (mean distance + x*std_dev)
    # will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    pcl_pt_filtered = outlier_filter.filter()

    # TODO: RANSAC Plane Segmentation
    table_seg = pcl_pt_filtered.make_segmenter()
    table_seg.set_model_type(pcl.SACMODEL_PLANE)
    table_seg.set_method_type(pcl.SAC_RANSAC)

    max_dist = 0.01
    table_seg.set_distance_threshold(max_dist)

    # TODO: Extract inliers and outliers
    inlier_ind, coeff = table_seg.segment() 
    table = pcl_pt_filtered.extract(inlier_ind, negative=False)   
    objects = pcl_pt_filtered.extract(inlier_ind, negative=True)

    # TODO: Euclidean Clustering
    objects_xyz = XYZRGB_to_XYZ(objects)

    tree = objects_xyz.make_kdtree()

    ec = objects_xyz.make_EuclideanClusterExtraction()
    
    # world 1: 0.025, 130, 1300
    # world 2: 0.02, 115, 1300
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(1300)

    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([objects_xyz[index][0],
                                             objects_xyz[index][1],
                                             objects_xyz[index][2],
                                             rgb_to_float(cluster_color[j])])
    
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    filtered_table = pcl_to_ros(table) 
    filtered_objects = pcl_to_ros(objects)
    clustered_objects = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(filtered_objects)
    pcl_table_pub.publish(filtered_table)
    pcl_cluster_pub.publish(clustered_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
#    try:
#        pr2_mover(detected_objects_list)
#    except rospy.ROSInterruptException:
#        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('pr2_pap', anonymous=True)

    # TODO: Create Subscribers
    pr2_pc_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    
    detected_obj_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)
    marker_pub = rospy.Publisher('object_markers', Marker, queue_size=1)
    # TODO: Create Publishers 
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    
    # TODO: Load Model From disk
    model = pickle.load(open('model_3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
