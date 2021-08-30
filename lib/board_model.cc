#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <sophus/so3.h>
#include <sophus/se3.h>
#include <string>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

std::vector<Eigen::Matrix4d> FindTagPose(cv::Mat K_,cv::Mat img_raw)
{
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  apriltag_detection_info_t info_;
  std::vector<Eigen::Matrix4d> poses;
  td_ = apriltag_detector_create();
  tf_ = tag36h11_create();
    // tf_ = tagStandard41h12_create();
    // tf_ = tagStandard52h13_create();
  td_->quad_decimate = 1.0;
  td_->quad_sigma = 0.0;
  td_->nthreads = 2;
  td_->debug = 0;
  td_->refine_edges = 1;

  info_.cx = K_.at<double>(0, 2);
  info_.cy = K_.at<double>(1, 2);
  info_.fx = K_.at<double>(0, 0);
  info_.fy = K_.at<double>(1, 1);
  apriltag_detector_add_family(td_, tf_);

  cv::Mat gray_img;
  cv::cvtColor(img_raw, gray_img, CV_BGR2GRAY);
  image_u8_t apriltag_image = {.width = gray_img.cols,
                               .height = gray_img.rows,
                               .stride = gray_img.cols,
                               .buf = gray_img.data};
  zarray_t *detections;
  detections = apriltag_detector_detect(td_, &apriltag_image);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    apriltag_pose_t pose;
    zarray_get(detections, i, &det);
    info_.tagsize = 0.4;
    info_.det = det;
    double err = estimate_tag_pose(&info_, &pose);
    Eigen::Matrix4d e_pose;
    e_pose<<pose.R->data[0],pose.R->data[1],pose.R->data[2],pose.t->data[0],
            pose.R->data[3],pose.R->data[4],pose.R->data[5],pose.t->data[1],
            pose.R->data[6],pose.R->data[7],pose.R->data[8],pose.t->data[2],
            0,0,0,1;
    poses.push_back(e_pose);
  }
  return poses;
}

int main(int argc, char const *argv[])
{
  /* code */
  std::string img_path = "/home/ramlab/Documents/matlab/single_board_cali/data/uncertainty/1619525819.650947.jpg";
  cv::Mat  K = (cv::Mat_<double>(3,3)<< 1342.26178254686,0,953.967220013334,0,1342.42450128927,551.355331517283,0,0,1); 
  cv::Mat D = (cv::Mat_<double>(1,5)<<-0.435968195000876, 0.224288180800522, 0.000218608552556113,-4.70308791162157e-05, -0.067235540213618);
  cv::Mat img = cv::imread(img_path);
  cv::Mat undist_img;
  cv::undistort(img,undist_img,K,D);
  cv::imshow("undistorted",undist_img);
  cv::waitKey(0);

  std::vector<apriltag_pose_t> tagsposes;
  std::vector<Eigen::Matrix<double,6,1>> se3s;
  for(int i=0;i<50;i++)
  {
    std::vector<Eigen::Matrix4d> tags_pose = FindTagPose(K,undist_img);
    if( tags_pose.size()>0) {
      Sophus::SE3 SE3_Rt(tags_pose[0].block<3,3>(0,0),tags_pose[0].block<3,1>(0,3));
      std::cout<<"se3: "<<SE3_Rt.log().transpose()<<std::endl;
    }
  }
  return 0;
}
