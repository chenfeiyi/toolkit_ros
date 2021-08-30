#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include "file_operation.hpp"
int main(int argc, char *argv[])
{
  ros::init(argc,argv,"kitti_lidar");
  ros::NodeHandle nh;
  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",10);
  std::string lidar_file_path  = "/media/ramlab/hdd3/dataset/kitti/lidar/sequences/13/velodyne/";
  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // read files from folder
  std::vector<std::string> file_list;
  calibrator_pipeline::common::GetFilelist(lidar_file_path,&file_list,false);
  ros::Rate rate(10);
  int idx=0;
  sensor_msgs::PointCloud2 pcd_msg;
  while(ros::ok()){
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
    pcl::toROSMsg(point_cloud,pcd_msg);

    std::vector<std::string> pcd_field;
    calibrator_pipeline::common::SplitString(file_list[idx],&pcd_field,'.');
    pcd_msg.header.stamp =
          ros::Time(std::atof((pcd_field[0] + '.' + pcd_field[1]).c_str()));
    pcd_msg.header.frame_id="velodyne";
    lidar_pub.publish(pcd_msg);

    idx++;
    if (idx>file_list.size())
    {
      break;
    }
    rate.sleep();
  }  
  return 0;
}
