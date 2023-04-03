'''
Author: windzu windzu1@gmail.com
Date: 2023-04-01 17:11:33
LastEditors: windzu windzu1@gmail.com
LastEditTime: 2023-04-03 10:23:17
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
from rich.progress import track


def read_calib_file(calib_file):
    with open(calib_file, 'r') as f:
        calib_dict = yaml.safe_load(f)

    transform_dict = {}
    for key in calib_dict:
        transform = calib_dict[key]
        transform_mat = np.eye(4)

        # rotation
        r = R.from_euler(
            'xyz', [transform['roll'], transform['pitch'], transform['yaw']],
            degrees=False)
        transform_mat[:3, :3] = r.as_matrix()

        # translation
        transform_mat[:3, 3] = [transform['x'], transform['y'], transform['z']]

        transform_dict[key] = transform_mat

    return transform_dict


def write_bag(calib_dict, input_bag, output_bag):

    # transform the point cloud and write to bag
    with rosbag.Bag(output_bag, 'w') as outbag:
        for cur_topic, msg, t in track(rosbag.Bag(input_bag).read_messages()):
            if cur_topic in calib_dict:
                # transform the point cloud
                transform = calib_dict[cur_topic]
                cloud = transform_cloud(msg, transform)
                # convert cloud to PointCloud2
                header = Header()
                header.stamp = t
                header.frame_id = "base_link"
                cloud_msg = pc2.create_cloud_xyz32(header, cloud)
                outbag.write(cur_topic, cloud_msg, t)
            else:
                outbag.write(cur_topic, msg, t)


def transform_cloud(cloud_msg, transform_mat):
    # convert PointCloud2 to numpy array
    cloud = pc2.read_points(
        cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
    cloud = np.array(list(cloud))
    cloud = np.hstack((cloud, np.ones((cloud.shape[0], 1))))

    cloud = np.dot(transform_mat, cloud.T).T

    # convert cloud to 3dim
    cloud = cloud[:, :3]
    return cloud


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Transform topics in a ROS bag file')
    parser.add_argument(
        '--calib',
        type=str,
        default='calib.yaml',
        help='Calibration file path')
    parser.add_argument(
        '--input',
        type=str,
        default='input.bag',
        help='Input ROS bag file path')
    parser.add_argument(
        '--output',
        type=str,
        default='output.bag',
        help='Output ROS bag file path')
    args = parser.parse_args()

    # print args
    print('Calibration file: ', args.calib)
    print('Input bag file: ', args.input)
    print('Output bag file: ', args.output)

    calib_dict = read_calib_file(args.calib)
    write_bag(calib_dict, args.input, args.output)
