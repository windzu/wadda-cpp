/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-04-02 23:34:25
 * @LastEditors: windzu windzu1@gmail.com
 * @LastEditTime: 2023-04-03 01:57:27
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

// cpp system headers
#include <iostream>
#include <string>
#include <vector>

// eigen headers
#include <Eigen/Eigen>

// pcl headers
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// ros headers
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

Eigen::Affine3f front_transform = Eigen::Affine3f::Identity();
Eigen::Affine3f left_transform = Eigen::Affine3f::Identity();
Eigen::Affine3f right_transform = Eigen::Affine3f::Identity();
Eigen::Affine3f top_transform = Eigen::Affine3f::Identity();

ros::Publisher pub;

void init_transform_matrix(Eigen::Affine3f* matrix, float x, float y, float z,
                           float roll, float pitch, float yaw) {
  (*matrix).translation() << x, y, z;
  (*matrix).rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                   Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
  return;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& front_msg,
              const sensor_msgs::PointCloud2ConstPtr& left_msg,
              const sensor_msgs::PointCloud2ConstPtr& right_msg,
              const sensor_msgs::PointCloud2ConstPtr& top_msg) {
  // convert sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr top_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*front_msg, *front_cloud);
  pcl::fromROSMsg(*left_msg, *left_cloud);
  pcl::fromROSMsg(*right_msg, *right_cloud);
  pcl::fromROSMsg(*top_msg, *top_cloud);

  // transform point cloud
  pcl::transformPointCloud(*front_cloud, *front_cloud, front_transform);
  pcl::transformPointCloud(*left_cloud, *left_cloud, left_transform);
  pcl::transformPointCloud(*right_cloud, *right_cloud, right_transform);
  pcl::transformPointCloud(*top_cloud, *top_cloud, top_transform);

  // merge point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  *merged_cloud += *front_cloud;
  *merged_cloud += *left_cloud;
  *merged_cloud += *right_cloud;
  *merged_cloud += *top_cloud;

  // convert pcl::PointCloud to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 merged_msg;
  pcl::toROSMsg(*merged_cloud, merged_msg);
  merged_msg.header.frame_id = "base_link";
  merged_msg.header.stamp = ros::Time::now();
  pub.publish(merged_msg);

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge_pc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string node_name = ros::this_node::getName();

  // get /lidar_points/front param
  std::vector<std::string> topic_list;
  std::vector<float> lidar_front_calib;
  std::vector<float> lidar_left_calib;
  std::vector<float> lidar_right_calib;
  std::vector<float> lidar_top_calib;
  pnh.getParam(node_name + "/topic_list", topic_list);
  pnh.getParam(node_name + "/lidar_front_calib", lidar_front_calib);
  pnh.getParam(node_name + "/lidar_left_calib", lidar_left_calib);
  pnh.getParam(node_name + "/lidar_right_calib", lidar_right_calib);
  pnh.getParam(node_name + "/lidar_top_calib", lidar_top_calib);

  // init transform matrix
  init_transform_matrix(&front_transform, lidar_front_calib[0],
                        lidar_front_calib[1], lidar_front_calib[2],
                        lidar_front_calib[3], lidar_front_calib[4],
                        lidar_front_calib[5]);
  init_transform_matrix(&left_transform, lidar_left_calib[0],
                        lidar_left_calib[1], lidar_left_calib[2],
                        lidar_left_calib[3], lidar_left_calib[4],
                        lidar_left_calib[5]);
  init_transform_matrix(&right_transform, lidar_right_calib[0],
                        lidar_right_calib[1], lidar_right_calib[2],
                        lidar_right_calib[3], lidar_right_calib[4],
                        lidar_right_calib[5]);
  init_transform_matrix(&top_transform, lidar_top_calib[0], lidar_top_calib[1],
                        lidar_top_calib[2], lidar_top_calib[3],
                        lidar_top_calib[4], lidar_top_calib[5]);

  // init message filter subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> front_sub(
      nh, topic_list[0], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> left_sub(
      nh, topic_list[1], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> right_sub(
      nh, topic_list[2], 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> top_sub(
      nh, topic_list[3], 10);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), front_sub, left_sub, right_sub, top_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // init publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_points", 10);

  ros::spin();
  return 0;
}
