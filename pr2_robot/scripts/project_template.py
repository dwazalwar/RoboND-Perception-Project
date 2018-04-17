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

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    noisy_data_pub.publish(pcl_msg)

    # Statistical Outlier Filtering
    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = pcl_data.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(25)

    # Set threshold scale factor
    x = .1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    filename = 'outlier_filter_output.pcd'
    pcl.save(cloud_filtered, filename)

    ros_filter_cloud = pcl_to_ros(cloud_filtered)
    filtered_data_pub.publish(ros_filter_cloud)

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)

    # PassThrough Filter along Z direction
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)

    # PassThrough Filter along Y direction to remove table edges
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)

    ros_passthrough_cloud = pcl_to_ros(cloud_filtered)
    passthrough_data_pub.publish(ros_passthrough_cloud)

    # RANSAC Plane Segmentation to separate table and its objects
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    filename = 'extracted_inliers.pcd'
    pcl.save(cloud_table, filename)

    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    filename = 'extracted_outliers.pcd'
    pcl.save(cloud_objects, filename)

    # Publish these objects
    ros_msg_objects = pcl_to_ros(cloud_objects)
    objects_pub.publish(ros_msg_objects)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)  # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to Experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene for Rviz visualization
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
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
    test_scene_num = Int32()
    test_scene_num.data = 1

    object_name = String()
    arm_name = String()

    pick_pose = Pose()
    place_pose = Pose()

    output_yaml = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')

    # Parse parameters into individual variables
    for target_object in object_list_param:
        object_name.data = target_object['name']

        # Loop through the pick list
        for object in object_list:
            if object.label == object_name.data:

                # Get the PointCloud for a given object and obtain it's centroid
                points_arr = ros_to_pcl(object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]

                # Recast to native Python float type
                scalar_centroid = [np.asscalar(element) for element in centroid]

                pick_pose.position.x = scalar_centroid[0]
                pick_pose.position.y = scalar_centroid[1]
                pick_pose.position.z = scalar_centroid[2]

                # Create 'place_pose' for the object
                place_pose.position.x = 0.0
                place_pose.position.z = 0.605

                # Assign the arm to be used for pick_place
                target_object_group = target_object['group']
                if target_object_group == "red":
                    arm_name.data = "left"
                    place_pose.position.y = 0.71
                else:
                    arm_name.data = "right"
                    place_pose.position.y = -0.71

                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                output_yaml.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))
                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                    # TODO: Insert your message variables to be sent as a service request
                    #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

                    #print ("Response: ", resp.success)

                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e

    # Output your request parameters into output yaml file
    filename = "output_file_" + str(test_scene_num.data) + ".yaml"
    send_to_yaml(filename, output_yaml)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('project3', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    noisy_data_pub = rospy.Publisher("/pcl_noisy_data", PointCloud2, queue_size=1)
    filtered_data_pub = rospy.Publisher("/pcl_filtered_data", PointCloud2, queue_size=1)
    passthrough_data_pub = rospy.Publisher("/pcl_passthrough_data", PointCloud2, queue_size=1)
    objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
