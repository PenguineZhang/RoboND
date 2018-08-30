#!/usr/bin/env python

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

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

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
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(1100)

    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

# Exercise-3 TODOs: 
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = objects.extract(pts_list)

        # Compute the associated feature vector
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster) 
        
        # TODO: complete this step just as is convered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(objects_xyz[pts_list[0]])
        label_pos[2] += .4
        marker_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels) )
    
    # Publish the list of detected objects
    detected_obj_pub.publish(detected_objects)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('classification', anonymous=True)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2, pcl_callback, queue_size=1 ) 

    # TODO: Create Publishers
    marker_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_obj_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

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
