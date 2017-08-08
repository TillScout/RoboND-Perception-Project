#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from features import compute_color_histograms
from features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose, Point
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


    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    pcl_data = vox.filter()
    
    # PassThrough Filter
    passthrough_z = pcl_data.make_passthrough_filter()
    filter_axis = "z"
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 0.8
    passthrough_z.set_filter_limits(axis_min, axis_max)
    pcl_data = passthrough_z.filter()
    
    passthrough_x = pcl_data.make_passthrough_filter()
    filter_axis = "x"
    passthrough_x.set_filter_field_name(filter_axis)
    axis_min = 0.35
    axis_max = 1.0
    passthrough_x.set_filter_limits(axis_min, axis_max)
    pcl_data = passthrough_x.filter()
    

    # RANSAC Plane Segmentation
    seg = pcl_data.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    #pcl_table = pcl_data.extract(inliers, negative=False)
    pcl_objects = pcl_data.extract(inliers, negative=True)
    
    # Remove noise:
    outlier_filter = pcl_objects.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(5)
    outlier_filter.set_std_dev_mul_thresh(0.)
    pcl_objects = outlier_filter.filter()


    # Euclidean Clustering
    white_objects = XYZRGB_to_XYZ(pcl_objects)
    

    
    tree = white_objects.make_kdtree()
    ec = white_objects.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_objects[indice][0],
                                            white_objects[indice][1],
                                            white_objects[indice][2],
                                             rgb_to_float(cluster_color[j])])
    
    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)


    # Convert PCL data to ROS messages
    ros_pcl_objects = pcl_to_ros(pcl_objects)
    ros_pcl_clusters = pcl_to_ros(cluster_cloud)
    # Publish ROS messages
    pcl_objects_pub.publish(ros_pcl_objects)
    cluster_test_pub.publish(ros_pcl_clusters)

# Exercise-3 TODOs:
    
    # Classify the clusters!
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        # pcl_cluster = cloud_out.extract(pts_list)
        pcl_cluster = pcl_objects.extract(pts_list)

        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(pcl_objects[pts_list[0]])
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
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    labels = []
    centroids = []
    groups = []
    names = []
    dict_list = []
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    drop_pose = Pose()

    # get parameters
    pick_list = rospy.get_param("/object_list")
    scene_number = rospy.get_param("world")
    dropbox = rospy.get_param("/dropbox")
    
    test_scene_num.data = scene_number

    # Parse parameters into individual variables
    for item in pick_list:
        groups.append(item["group"])
        names.append(item["name"])
    
    if dropbox[0]["group"] == "red":
        rx, ry, rz = dropbox[0]["position"]
        gx, gy, gz = dropbox[1]["position"]
    else:
        rx, ry, rz = dropbox[1]["position"]
        gx, gy, gz = dropbox[0]["position"]
    

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the object list
    for item in object_list:
        labels.append(item.label)
        # Get the PointCloud for a given object and obtain it's centroid
        points_arr = ros_to_pcl(item.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    
    objects = dict(zip(labels, centroids))
    
    # loop through pick list
    for item in pick_list:
        if item["name"] in labels:
            
            name = item["name"]
            group = item["group"]
            object_name.data = str(name)
            
            if group == "green":
                arm_name.data = "right"
                drop_pose.position = Point(gx, gy, gz)
            else:
                arm_name.data = "left"
                drop_pose.position = Point(rx, ry, rz)
            
            centroid = objects[name]
            x,y,z = tuple(centroid)
            pick_pose.position = Point(float(x),float(y),float(z))
        
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, drop_pose)
            dict_list.append(yaml_dict)



    # Output your request parameters into output yaml file
    yaml_filename = "result_world_" + str(scene_number)+".yaml"
    send_to_yaml(yaml_filename, dict_list)

"""
        
        
        
        
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
"""


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    cluster_test_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # load model from disk
    model = pickle.load(open('model_project.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []
    

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
