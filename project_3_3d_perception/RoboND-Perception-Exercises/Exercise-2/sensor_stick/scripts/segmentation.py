#!/usr/bin/env python

# Import modules
from pcl_helper import *
import pcl
# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
#    print type(pcl_data)
    
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
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(115)
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


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2, pcl_callback, queue_size=1) 
    
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
