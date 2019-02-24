#include <iostream>
#include <string>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include "rosbag/bag.h"

using namespace sensor_msgs;
using namespace ros::topic;
using namespace tf;
using namespace std;

void print_usage() { 
  cout << "Saves a point cloud on head_camera/depth_registered/points to "
               "NAME.bag in the current directory."
            << endl;
  cout << "Usage: rosrun perception save_cloud NAME" << endl;
}

int transformCloud(const PointCloud2& cloud_in, PointCloud2& cloud_out, const string& target_frame) {
  TransformListener tf_listener;
  tf_listener.waitForTransform(target_frame, cloud_in.header.frame_id, ros::Time(0), ros::Duration(5.0));
  StampedTransform transform;

  try {
    tf_listener.lookupTransform(target_frame, cloud_in.header.frame_id, ros::Time(0), transform);
  } catch (LookupException& e) {
    cerr << e.what() << endl;
    return 1;
  } catch (tf::ExtrapolationException& e) {
    std:cerr << e.what() << endl;
    return 1;
  }

  pcl_ros::transformPointCloud("base_link", transform, cloud_in, cloud_out);
  return 0;
}

void saveCloudToFile(string& filename, string& cloud_topic, PointCloud2& cloud) {
  filename += ".bag";
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  bag.write(cloud_topic, ros::Time::now(), cloud);
  bag.close();
}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "save_cloud_main");
  if (argc < 2) { 
    print_usage();
    return 1;
  } 
  string cloud_topic("cloud");

  PointCloud2ConstPtr cloud = waitForMessage<PointCloud2>(cloud_topic);
  PointCloud2 cloud_out;
  if (transformCloud(*cloud, cloud_out, "base_link")) return 1;

  string filename(argv[1]);
  saveCloudToFile(filename, cloud_topic, cloud_out);

  return 0;
}
