/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-21 21:59:46
 * @Description: content
 */
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
int main(int argc, char *argv[])
{
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ,pcl::_Normal> norm_est;
  norm_est.setSearchMethod()
  return 0;
}
