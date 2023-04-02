'''
Author: windzu windzu1@gmail.com
Date: 2023-04-01 17:11:33
LastEditors: windzu windzu1@gmail.com
LastEditTime: 2023-04-02 23:35:17
Description: 
Copyright (c) 2023 by windzu, All Rights Reserved. 
'''
import rosbag
import yaml
import rospy
import message_filters
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import struct
from scipy.spatial.transform import Rotation as R

def read_calib_file(calib_file):
    with open(calib_file, 'r') as f:
        calib_dict = yaml.safe_load(f)
    
    transform_dict = {}
    for key in calib_dict:
        transform = calib_dict[key]
        transform_mat=np.eye(4)

        # rotation
        r=R.from_euler('xyz', [transform['roll'], transform['pitch'], transform['yaw']], degrees=False)
        transform_mat[:3,:3]=r.as_matrix()

        # translation
        transform_mat[:3,3]=[transform['x'], transform['y'], transform['z']]

        transform_dict[key] = transform_mat
    
    return transform_dict

            
def write_bag(calib_dict):
    input_topics = list(calib_dict.keys())
    # print input_topics
    print("input topics: ", input_topics)
    sub_list = []
    for topic in input_topics:
        sub = message_filters.Subscriber(topic, PointCloud2)
        sub_list.append(sub)
    
    sync = message_filters.ApproximateTimeSynchronizer(sub_list, queue_size=10,slop=0.1)
    sync.registerCallback(callback)

    rospy.spin()

def callback(cloud0,cloud1,cloud2,cloud3):
    print("callback")
    cloud_msg_list = [cloud0,cloud1,cloud2,cloud3]

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"

    transformed_clouds = []

    for cloud_msg in cloud_msg_list:
        topic = cloud_msg._connection_header['topic']
        if topic in calib_dict:
            cloud=transform_cloud(cloud_msg, calib_dict[topic])
            transformed_clouds.append(cloud)

    merged_cloud = np.vstack(transformed_clouds)

    # convert numpy array to PointCloud2
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]

    merged_cloud_msg = pc2.create_cloud(header, fields, merged_cloud)
    # publish the merged point cloud
    pub.publish(merged_cloud_msg)


def transform_cloud(cloud_msg, transform_mat):
    # convert PointCloud2 to numpy array
    cloud = pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
    cloud = np.array(list(cloud))
    cloud = np.hstack((cloud, np.ones((cloud.shape[0], 1))))

    cloud = np.dot(transform_mat, cloud.T).T

    #convert cloud to 3dim
    cloud = cloud[:, :3]
    return cloud


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Transform topics in a ROS bag file')
    parser.add_argument('--calib', type=str, default='calib.yaml', help='Calibration file path')
    parser.add_argument('--input', type=str, default='input.bag', help='Input ROS bag file path')
    parser.add_argument('--output', type=str, default='output.bag', help='Output ROS bag file path')
    args = parser.parse_args()

    calib_dict = read_calib_file(args.calib)
    print(calib_dict)


    rospy.init_node('transform_pointclouds')
    pub = rospy.Publisher('/lidar_points', PointCloud2, queue_size=10)
    write_bag(calib_dict)

