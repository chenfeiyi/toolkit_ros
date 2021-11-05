#include <iostream>
#include <vector>
#include <string>
#include "BagReader.h"

int main(int argc, char *argv[])
{
  std::string bag_path = "/media/ramlab/SolidDisk/bbc.bag";
  std::string img_save_path = "/media/ramlab/SolidDisk/zuo/";
  std::string pcd_save_path = "/media/ramlab/SolidDisk/zuo/";

  BagReader bag_reader;
  bag_reader.LoadFromFile(bag_path);
  bag_reader.SaveData(img_save_path, "sensor_msgs/Image");
  // bag_reader.SaveData(pcd_save_path, "sensor_msgs/PointCloud2");
  // bag_reader.SaveData(pcd_save_path, "sensor_msgs/PointCloud2");
  return 0;
}
