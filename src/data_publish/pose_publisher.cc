/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-14 17:19:14
 * @Description: content
 */
#include <assert.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>

#include "../common/file_operation.hpp"
#include "cmdline.h"
#include "ros/ros.h"
using namespace std;

typedef struct checkerborpose {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} checkerborpose;

/** 
 * @brief: play poses stored in the file using multi-float array
 */

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "img_player");
  ros::NodeHandle nh;

  cmdline::parser a;
  a.add<bool>("loop", 'l', "play data repeatly", false, false);
  a.add<float>("hz", 'z', "frequency to paly the data", false, 0.5);
  a.parse_check(argc, argv);
  ros::Publisher pose_pub =
      nh.advertise<std_msgs::Float32MultiArray>("/checkerctrl", 1);
  std::string pose_file =
      "/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-camera/"
      "pose.txt";
  std::vector<std::string> fields;
  std::vector<checkerborpose> poses;
  std::fstream fin;
  fin.open(pose_file.c_str(), std::ios::in);
  if (!fin.is_open()) {
    std::cout << "can't open file " << pose_file.c_str() << std::endl;
  }
  std::string line;
  while (std::getline(fin, line)) {
    checkerborpose pose;
    //  将line一整句string按照分号切割成多个string并存储到fields中vector中;
    fields.clear();
    calibrator_pipeline::common::SplitString(line, &fields, ' ');
    for (int i = 0; i < fields.size(); i++) {
      std::cout << i << ": " << fields[i] << std::endl;
    }
    pose.x = stof(fields[0]);
    pose.y = stof(fields[1]);
    pose.z = stof(fields[2]);
    pose.roll = stof(fields[3]);
    pose.pitch = stof(fields[4]);
    pose.yaw = stof(fields[5]);
    poses.push_back(pose);
  }
  fin.close();
  ros::Rate rate(a.get<float>("hz"));
  std::cout << "******************************" << std::endl;
  std::cout << "start to play data" << std::endl;
  int i = 0;
  while (ros::ok()) {
    std_msgs::Float32MultiArray pose_set;
    pose_set.data.push_back(poses[i].x);
    pose_set.data.push_back(poses[i].y);
    pose_set.data.push_back(poses[i].z);
    pose_set.data.push_back(poses[i].roll);
    pose_set.data.push_back(poses[i].pitch);
    pose_set.data.push_back(poses[i].yaw);
    pose_pub.publish(pose_set);
    i++;
    rate.sleep();
    ros::spinOnce();
    if (i >= poses.size()) {
      break;
    }
  }
  rate.sleep();
  rate.sleep();
  rate.sleep();
  std::cout << "Done!" << std::endl;
  return 0;
}
