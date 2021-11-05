#include "BagReader.h"
BagReader::BagReader(/* args */) { bag_ = new rosbag::Bag; }

BagReader::~BagReader() { bag_->close(); }
bool BagReader::LoadFromFile(std::string bagFilePath) {
  bag_->open(bagFilePath, rosbag::bagmode::Read);
  rosbag::View view(*bag_);
  size_t idx = 1;
  BOOST_FOREACH (rosbag::MessageInstance m, view) {
    auto it = topicMap_.find(m.getTopic());
    if (it == topicMap_.end()) {
      topicMap_.emplace(m.getTopic(), idx);
      idx++;
    }
  }
}
void BagReader::SaveData(std::string folderPath, std::string dataType) {
  rosbag::View view(*bag_);
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  BOOST_FOREACH (rosbag::MessageInstance m, view) {
    if (m.getDataType() == "sensor_msgs/Image" &&
        (dataType == "sensor_msgs/Image" || dataType == "all")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;

      sensor_msgs::Image::ConstPtr imgMsg = m.instantiate<sensor_msgs::Image>();
      cv_image_ptr =
          cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
      cv_image = cv_image_ptr->image;
      ss << folderPath << topicMap_[m.getTopic()];

      if (access(ss.str().c_str(), 0) == -1) {
        mkdir(ss.str().c_str(), 0777);
      }
      ss << "/" << std::fixed << std::setprecision(6)
         << imgMsg->header.stamp.toSec() << ".jpg";
      std::cout << "Save file: " << ss.str() << std::endl;
      cv::imwrite(ss.str(), cv_image, compression_params);

    } else if (m.getDataType() == "sensor_msgs/CompressedImage" &&
               (dataType == "sensor_msgs/CompressedImage" ||
                dataType == "all")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;

      sensor_msgs::CompressedImage::ConstPtr imgMsg =
          m.instantiate<sensor_msgs::CompressedImage>();
      cv_image_ptr =
          cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
      cv_image = cv_image_ptr->image;
      ss << folderPath << topicMap_[m.getTopic()];

      if (access(ss.str().c_str(), 0) == -1) {
        mkdir(ss.str().c_str(), 0777);
      }
      ss << "/" << std::fixed << std::setprecision(6)
         << imgMsg->header.stamp.toSec() << ".jpg";
      std::cout << "Save file: " << ss.str() << std::endl;
      cv::imwrite(ss.str(), cv_image, compression_params);
    } else if (m.getDataType() == "sensor_msgs/PointCloud2" &&
               (dataType == "sensor_msgs/PointCloud2" || dataType == "all")) {
      sensor_msgs::PointCloud2::ConstPtr pcdMsg =
          m.instantiate<sensor_msgs::PointCloud2>();
      std::stringstream ss;
      ss << folderPath << topicMap_[m.getTopic()];
      if (access(ss.str().c_str(), 0) == -1) {
        mkdir(ss.str().c_str(), 0777);
      }
      ss << "/" << std::fixed << std::setprecision(6)
         << pcdMsg->header.stamp.toSec() << ".pcd";
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*pcdMsg, cloud);
      std::cout << "Save file: " << ss.str() << std::endl;
      cloud.width = cloud.width*cloud.height;
      cloud.height = 1;
      pcl::io::savePCDFile(ss.str(), cloud);
    }
  }
}

void BagReader::SaveData(std::string folderPath, std::string dataType,
                         std::string dataTopic) {
  rosbag::View view(*bag_);
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //选择jpeg
  compression_params.push_back(100);  //在这个填入你要的图片质量
  if (access(folderPath.c_str(), 0) == -1) {
    mkdir(folderPath.c_str(), 0777);
  }
  BOOST_FOREACH (rosbag::MessageInstance m, view) {
    if (m.getDataType() == "sensor_msgs/Image" &&
        "sensor_msgs/Image" == dataType && m.getTopic() == dataTopic) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;

      sensor_msgs::Image::ConstPtr imgMsg = m.instantiate<sensor_msgs::Image>();
      cv_image_ptr =
          cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
      cv_image = cv_image_ptr->image;

      ss << folderPath << std::fixed << std::setprecision(6)
         << imgMsg->header.stamp.toSec() << ".jpg";
      std::cout << "Save file: " << ss.str() << std::endl;
      cv::imwrite(ss.str(), cv_image, compression_params);

    } else if (m.getDataType() == "sensor_msgs/CompressedImage" &&
               dataType == "sensor_msgs/CompressedImage" &&
               m.getTopic() == dataTopic) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;

      sensor_msgs::CompressedImage::ConstPtr imgMsg =
          m.instantiate<sensor_msgs::CompressedImage>();
      cv_image_ptr =
          cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
      cv_image = cv_image_ptr->image;
      ss << folderPath << std::fixed << std::setprecision(6)
         << imgMsg->header.stamp.toSec() << ".jpg";
      std::cout << "Save file: " << ss.str() << std::endl;
      cv::imwrite(ss.str(), cv_image, compression_params);
    } else if (m.getDataType() == "sensor_msgs/PointCloud2" &&
               dataType == "sensor_msgs/PointCloud2" &&
               m.getTopic() == dataTopic) {
      sensor_msgs::PointCloud2::ConstPtr pcdMsg =
          m.instantiate<sensor_msgs::PointCloud2>();
      std::stringstream ss;
      ss << folderPath << std::fixed << std::setprecision(6)
         << m.getTime().toSec() << ".pcd";
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*pcdMsg, cloud);
      std::cout << "Save file: " << ss.str() << std::endl;
      pcl::io::savePCDFile(ss.str(), cloud);
    }
  }
}
void BagReader::SaveData(std::string folderPath, std::string dataType,
                         std::string dataTopic, std::vector<double> timeList) {
  rosbag::View view(*bag_);
  std::vector<int> compression_params;
  int timeIdx = 0;
  bool saveOrNotSave = false;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //选择jpeg
  compression_params.push_back(100);  //在这个填入你要的图片质量
  if (access(folderPath.c_str(), 0) == -1) {
    mkdir(folderPath.c_str(), 0777);
  }
  BOOST_FOREACH (rosbag::MessageInstance m, view) {
    if (m.getDataType() == "sensor_msgs/Image" &&
        "sensor_msgs/Image" == dataType &&
        (m.getTopic() == dataTopic || dataTopic == "null")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;
      sensor_msgs::Image::ConstPtr imgMsg = m.instantiate<sensor_msgs::Image>();
      saveOrNotSave = false;
      while (timeIdx < timeList.size()) {
        double timeGap = imgMsg->header.stamp.toSec() - timeList[timeIdx];
        if (fabs(timeGap) < 0.001) {
          timeIdx++;
          saveOrNotSave = true;
          break;
        } else if (timeGap > 0.001) {
          timeIdx++;
          saveOrNotSave = false;
          continue;
        } else {
          saveOrNotSave = false;
          break;
        }
      }
      if (saveOrNotSave) {
        cv_image_ptr =
            cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_image_ptr->image;

        ss << folderPath << std::fixed << std::setprecision(6)
           << imgMsg->header.stamp.toSec() << ".jpg";
        std::cout << "Save file: " << ss.str() << std::endl;
        cv::imwrite(ss.str(), cv_image, compression_params);
      }

    } else if (m.getDataType() == "sensor_msgs/CompressedImage" &&
               dataType == "sensor_msgs/CompressedImage" &&
               (m.getTopic() == dataTopic || dataTopic == "null")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;
      saveOrNotSave = false;
      sensor_msgs::CompressedImage::ConstPtr imgMsg =
          m.instantiate<sensor_msgs::CompressedImage>();
      while (timeIdx < timeList.size()) {
        double timeGap = imgMsg->header.stamp.toSec() - timeList[timeIdx];
        if (fabs(timeGap) < 0.001) {
          timeIdx++;
          saveOrNotSave = true;
          break;
        } else if (timeGap > 0.001) {
          timeIdx++;
          saveOrNotSave = false;
          continue;
        } else {
          saveOrNotSave = false;
          break;
        }
      }
      if (saveOrNotSave) {
        cv_image_ptr =
            cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_image_ptr->image;
        ss << folderPath << std::fixed << std::setprecision(6)
           << imgMsg->header.stamp.toSec() << ".jpg";
        std::cout << "Save file: " << ss.str() << std::endl;
        cv::imwrite(ss.str(), cv_image, compression_params);
      }

    } else if (m.getDataType() == "sensor_msgs/PointCloud2" &&
               dataType == "sensor_msgs/PointCloud2" &&
               (m.getTopic() == dataTopic || dataTopic == "null")) {
      saveOrNotSave = false;
      sensor_msgs::PointCloud2::ConstPtr pcdMsg =
          m.instantiate<sensor_msgs::PointCloud2>();
      while (timeIdx < timeList.size()) {
        double timeGap = pcdMsg->header.stamp.toSec() - timeList[timeIdx];
        if (fabs(timeGap) < 0.001) {
          timeIdx++;
          saveOrNotSave = true;
          break;
        } else if (timeGap > 0.001) {
          timeIdx++;
          saveOrNotSave = false;
          continue;
        } else {
          saveOrNotSave = false;
          break;
        }
      }
      if (saveOrNotSave) {
        std::stringstream ss;
        ss << folderPath << std::fixed << std::setprecision(6)
           << pcdMsg->header.stamp.toSec() << ".pcd";
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*pcdMsg, cloud);
        std::cout << "Save file: " << ss.str() << std::endl;
        pcl::io::savePCDFile(ss.str(), cloud);
      }
    }
  }
}
void BagReader::SaveImage(std::string folderPath, std::string dataType,
                          std::string dataTopic, std::vector<double> timeList,
                          Eigen::Matrix3d &K, Eigen::Matrix<double, 5, 1> &D) {
  rosbag::View view(*bag_);
  std::vector<int> compression_params;
  int timeIdx = 0;
  bool saveOrNotSave = false;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //选择jpeg
  compression_params.push_back(100);  //在这个填入你要的图片质量
  if (access(folderPath.c_str(), 0) == -1) {
    mkdir(folderPath.c_str(), 0777);
  }
  BOOST_FOREACH (rosbag::MessageInstance m, view) {
    if (m.getDataType() == "sensor_msgs/Image" &&
        "sensor_msgs/Image" == dataType &&
        (m.getTopic() == dataTopic || dataTopic == "null")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;
      sensor_msgs::Image::ConstPtr imgMsg = m.instantiate<sensor_msgs::Image>();
      saveOrNotSave = false;
      while (timeIdx < timeList.size()) {
        double timeGap = imgMsg->header.stamp.toSec() - timeList[timeIdx];
        if (fabs(timeGap) < 0.001) {
          timeIdx++;
          saveOrNotSave = true;
          break;
        } else if (timeGap > 0.001) {
          timeIdx++;
          saveOrNotSave = false;
          continue;
        } else {
          saveOrNotSave = false;
          break;
        }
      }
      if (saveOrNotSave) {
        cv_image_ptr =
            cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_image_ptr->image;

        ss << folderPath << std::fixed << std::setprecision(6)
           << imgMsg->header.stamp.toSec() << ".jpg";
        std::cout << "Save file: " << ss.str() << std::endl;
        cv::Mat K_cv, D_cv;
        cv::eigen2cv(K, K_cv);
        cv::eigen2cv(D, D_cv);
        cv::undistort(cv_image.clone(), cv_image, K_cv, D_cv);
        cv::imwrite(ss.str(), cv_image, compression_params);
      }

    } else if (m.getDataType() == "sensor_msgs/CompressedImage" &&
               dataType == "sensor_msgs/CompressedImage" &&
               (m.getTopic() == dataTopic || dataTopic == "null")) {
      cv::Mat cv_image;
      std::stringstream ss;
      cv_bridge::CvImagePtr cv_image_ptr;
      saveOrNotSave = false;
      sensor_msgs::CompressedImage::ConstPtr imgMsg =
          m.instantiate<sensor_msgs::CompressedImage>();
      while (timeIdx < timeList.size()) {
        double timeGap = imgMsg->header.stamp.toSec() - timeList[timeIdx];
        if (fabs(timeGap) < 0.001) {
          timeIdx++;
          saveOrNotSave = true;
          break;
        } else if (timeGap > 0.001) {
          timeIdx++;
          saveOrNotSave = false;
          continue;
        } else {
          saveOrNotSave = false;
          break;
        }
      }
      if (saveOrNotSave) {
        cv_image_ptr =
            cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_image_ptr->image;
        ss << folderPath << std::fixed << std::setprecision(6)
           << imgMsg->header.stamp.toSec() << ".jpg";
        std::cout << "Save file: " << ss.str() << std::endl;
        cv::Mat K_cv, D_cv;
        cv::eigen2cv(K, K_cv);
        cv::eigen2cv(D, D_cv);
        cv::undistort(cv_image.clone(), cv_image, K_cv, D_cv);
        cv::imwrite(ss.str(), cv_image, compression_params);
      }
    }
  }
}
std::vector<std::string> BagReader::getTopicList() {
  std::vector<std::string> topicList;
  for (auto it : topicMap_) {
    topicList.push_back(it.first);
  }
  return topicList;
}