#include <iostream>
#include <vector>
#include <string>
#include "../../common/BagReader.h"
/** 
 * @brief: extract message from bag files
 */
int main(int argc, char *argv[])
{
  std::string bag_path = "/home/ramlab/gazebo_cam_lidar.bag";
  std::string img_save_path = "/home/ramlab/Desktop/gazebo_test/";
  std::string pcd_save_path = "/home/ramlab/Desktop/gazebo_test/";

  BagReader bag_reader;
  bag_reader.LoadFromFile(bag_path);
  bag_reader.SaveData(img_save_path, "sensor_msgs/Image");
  // bag_reader.SaveData(pcd_save_path, "sensor_msgs/PointCloud2");
  bag_reader.SaveData(pcd_save_path, "sensor_msgs/PointCloud2");
  return 0;
}
