#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include "file_operation.hpp"

/** 
 * @brief: convert the bin type point cloud into pcd file
 */
int main(int argc, char *argv[])
{
  std::string lidar_file_path  = "/media/ramlab/hdd3/dataset/kitti/lidar/sequences/00/velodyne/";
  std::string save_path  ="/media/ramlab/hdd3/dataset/kitti/lidar/sequences/00/pcd/";
  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));
  std::vector<std::string> file_list;
  calibrator_pipeline::common::GetFilelist(lidar_file_path,&file_list,false);
  int idx=0;
  while(true){
    std::string file_name = lidar_file_path + file_list[idx];
    std::cout<<"file name: "<<file_name<<std::endl;
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;
    num = 1000000;
    // load point cloud
    FILE *stream;
    stream = fopen (file_name.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++) {
      pcl::PointXYZI pt;
      pt.x = *px;
      pt.y = *py;
      pt.z = *pz;
      pt.intensity = *pr;
      point_cloud.points.push_back(pt);
      px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);
    std::vector<std::string> pcd_field;
    calibrator_pipeline::common::SplitString(file_list[idx],&pcd_field,'.');
    point_cloud.height = point_cloud.size();
    point_cloud.width = 1;
    pcl::io::savePCDFile(save_path+pcd_field[0]+".pcd",point_cloud);
    idx++;
    if (idx>file_list.size())
    {
      break;
    }
  }
  return 0;
}
