/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-15 11:29:23
 * @Description: content
 */
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <fstream>
std::ofstream *file_out;
void odom_callback(nav_msgs::OdometryConstPtr laserOdometry) {
  Eigen::Quaterniond q_wodom_curr;
  Eigen::Vector3d t_wodom_curr;
  q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
  q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
  q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
  q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
  t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
  t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
  t_wodom_curr.z() = laserOdometry->pose.pose.position.z;
  *file_out << std::setprecision(6)
            << std::to_string(laserOdometry->header.stamp.toSec() - 965.765679)
            << " " << std::setprecision(7) << t_wodom_curr[0] << " "
            << t_wodom_curr[1] << " " << t_wodom_curr[2] << " "
            << q_wodom_curr.w() << " " << q_wodom_curr.x() << " "
            << q_wodom_curr.y() << " " << q_wodom_curr.z() << " " << std::endl;
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odom_recorder");
  ros::NodeHandle nh;
  file_out = new std::ofstream;
  file_out->open(
      "/home/ramlab/Documents/ros_workspace/files/lidar_odom.txt",
      std::ios::out | std::ios::trunc);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init", 10, odom_callback);
  ros::spin();
  file_out->close();
  return 0;
}
