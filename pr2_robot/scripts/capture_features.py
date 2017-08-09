#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from training_helper import spawn_model
from training_helper import delete_model
from training_helper import initial_setup
from training_helper import capture_sample
from features import compute_color_histograms
from features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from time import time

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'biscuits',
       'soap',
       'soap2',
       'book',
       'eraser',
       'glue',
       'snacks',
       'sticky_notes'
       ]

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []
    bad_samples = 0

    for model_name in models:
        spawn_model(model_name)
        num_attempts = 5000
        for i in range(num_attempts):
            # make five attempts to get a valid a point cloud then give up
            print "capturing", model_name, "attempt",i+1,"out of", num_attempts
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:

                sample_cloud = capture_sample()
                
                # test: try downsampling before capturing:
                pcl_data = ros_to_pcl(sample_cloud)
                vox = pcl_data.make_voxel_grid_filter()
                leaf_size = 0.01
                vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
                sample_cloud = vox.filter()
               
                sample_cloud_arr = sample_cloud.to_array()
                sample_cloud = pcl_to_ros(sample_cloud)
                
                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    bad_samples += 1
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            chists = compute_color_histograms(sample_cloud, using_hsv=True)
            normals = get_normals(sample_cloud)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])

        delete_model()

    print "number of bad samples:", bad_samples
    #pickle.dump(labeled_features, open('training_set.sav', 'wb'))
    pickle.dump(labeled_features, open('training_set_project.sav', 'wb'))
