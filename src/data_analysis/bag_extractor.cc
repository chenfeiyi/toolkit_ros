#include <iostream>
#include <vector>
#include <string>
#include "cmdline.h"
#include "../../common/BagReader.h"
/** 
 * @brief: extract message from bag files
 */
int main(int argc, char *argv[])
{
  cmdline::parser a;
  a.add<std::string>("bagpath", 'b', "bag path", true);
  a.add<std::string>("savepath", 's', "save path", true);
  a.parse_check(argc, argv);

  std::string bag_path = a.get<std::string>("bagpath");
  std::string save_path = a.get<std::string>("savepath");

  BagReader bag_reader;
  bag_reader.LoadFromFile(bag_path);
  bag_reader.SaveData(save_path, "sensor_msgs/Image");
  bag_reader.SaveData(save_path, "sensor_msgs/PointCloud2");
  bag_reader.SaveData(save_path, "sensor_msgs/CompressedImage");
  return 0;
}
