#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int argc, char const *argv[]) {
  Eigen::Matrix3d K;
  Eigen::Matrix<double, 5, 1> dist;

  K << 897.4566, 0, 635.4040, 0, 896.7992, 375.3149, 0, 0, 1;
  dist << -0.4398, 0.2329, -0.0011, 2.0984e-04, -0.0730;
  cv::Mat intrinsic = (cv::Mat_<float>(3, 3) << K(0, 0), K(0, 1), K(0, 2),
                       K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2));
  cv::Mat distort = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);

  std::vector<cv::Point3f> board_pts;
  std::vector<cv::Point2f> corners;
corners.emplace_back(580,174);
corners.emplace_back(770,305);
corners.emplace_back(622,545);
corners.emplace_back(411,408);
corners.emplace_back(563,121);
corners.emplace_back(830,302);
corners.emplace_back(603,645);
corners.emplace_back(316,444);
corners.emplace_back(391,113);
corners.emplace_back(652,303);
corners.emplace_back(476,648);
corners.emplace_back(132,468);
corners.emplace_back(894,109);
corners.emplace_back(1221,297);
corners.emplace_back(894,668);
corners.emplace_back(644,431);
corners.emplace_back(1003,154);
corners.emplace_back(1237,319);
corners.emplace_back(975,567);
corners.emplace_back(796,388);
corners.emplace_back(798,165);
corners.emplace_back(959,319);
corners.emplace_back(827,556);
corners.emplace_back(638,401);
corners.emplace_back(678,168);
corners.emplace_back(821,380);
corners.emplace_back(555,552);
corners.emplace_back(429,342);
corners.emplace_back(465,180);
corners.emplace_back(596,368);
corners.emplace_back(380,553);
corners.emplace_back(211,346);
corners.emplace_back(369,173);
corners.emplace_back(511,369);
corners.emplace_back(279,567);
corners.emplace_back(87,350);
corners.emplace_back(397,202);
corners.emplace_back(511,356);
corners.emplace_back(335,509);
corners.emplace_back(191,343);
corners.emplace_back(664,201);
corners.emplace_back(759,364);
corners.emplace_back(572,491);
corners.emplace_back(501,335);
corners.emplace_back(827,199);
corners.emplace_back(940,360);
corners.emplace_back(756,506);
corners.emplace_back(635,339);

board_pts.emplace_back(1.4481, 2.0291, 0.46446);
board_pts.emplace_back(1.8845, 1.7317, 0.12832);
board_pts.emplace_back(1.4579, 1.8193, -0.50304);
board_pts.emplace_back(1.0215, 2.1167, -0.1669);
board_pts.emplace_back(1.0438, 1.518, 0.43126);
board_pts.emplace_back(1.426, 1.1628, 0.085344);
board_pts.emplace_back(1.0132, 1.3275, -0.5398);
board_pts.emplace_back(0.63111, 1.6828, -0.19388);
board_pts.emplace_back(0.7474, 1.6555, 0.42176);
board_pts.emplace_back(1.255, 1.4894, 0.095209);
board_pts.emplace_back(0.83263, 1.4557, -0.54414);
board_pts.emplace_back(0.32504, 1.6218, -0.21759);
board_pts.emplace_back(1.4624, 1.0447, 0.42675);
board_pts.emplace_back(1.6032, 0.5561, 0.061612);
board_pts.emplace_back(1.3268, 0.93159, -0.54741);
board_pts.emplace_back(1.186, 1.4202, -0.18227);
board_pts.emplace_back(2.1078, 1.1898, 0.46997);
board_pts.emplace_back(2.2135, 0.73152, 0.056861);
board_pts.emplace_back(1.9993, 1.1967, -0.51407);
board_pts.emplace_back(1.8936, 1.655, -0.10096);
board_pts.emplace_back(1.8164, 1.5773, 0.47067);
board_pts.emplace_back(2.2841, 1.4068, 0.091006);
board_pts.emplace_back(1.8173, 1.4735, -0.51392);
board_pts.emplace_back(1.3496, 1.644, -0.13425);
board_pts.emplace_back(1.52, 1.7066, 0.44627);
board_pts.emplace_back(1.708, 1.412, -0.073101);
board_pts.emplace_back(1.2793, 1.8861, -0.49714);
board_pts.emplace_back(1.0914, 2.1807, 0.022239);
board_pts.emplace_back(1.201, 2.2062, 0.45227);
board_pts.emplace_back(1.5706, 2.0946, -0.040511);
board_pts.emplace_back(0.96258, 2.1373, -0.50616);
board_pts.emplace_back(0.593, 2.249, -0.013382);
board_pts.emplace_back(0.95674   ,    2.2583,      0.44321);
board_pts.emplace_back(1.3326    ,   2.1701 ,   -0.049585);
board_pts.emplace_back(0.72377   ,    2.1816,     -0.51597);
board_pts.emplace_back(0.34793   ,    2.2698,     -0.02317);
board_pts.emplace_back(1.2642    ,   2.7973 ,     0.48561);
board_pts.emplace_back(1.6469    ,   2.7104 ,  -0.0021055);
board_pts.emplace_back(1.0444    ,   2.7203 ,    -0.47664);
board_pts.emplace_back(0.66172   ,    2.8072,     0.011084);
board_pts.emplace_back(1.9271    ,    2.209 ,     0.48637);
board_pts.emplace_back(1.9908    ,    1.878 ,   -0.041103);
board_pts.emplace_back(1.7445    ,   2.4795 ,    -0.44835);
board_pts.emplace_back(1.6808    ,   2.8105 ,    0.079124);
board_pts.emplace_back(2.3015    ,   1.8777 ,     0.49492);
board_pts.emplace_back(2.6041    ,   1.6632 ,  -0.0093139);
board_pts.emplace_back(2.0379    ,     1.92 ,    -0.45844);
board_pts.emplace_back(1.7352    ,   2.1345 ,     0.04579);
  cv::Mat rvec, tvec;
  cv::Mat rotm;
  bool isSolvePnP =
  cv::solvePnP(board_pts, corners, intrinsic, distort, rvec, tvec, false);

  // cv::solvePnPRansac(board_pts, corners, intrinsic, distort, rvec, tvec);
  Eigen::Matrix3d rotm_ei;
  Eigen::Matrix4d transformation;
  Eigen::Vector3d t_ei;
  cv::Rodrigues(rvec, rotm);
  cv::cv2eigen(rotm, rotm_ei);
  cv::cv2eigen(tvec, t_ei);
  std::vector<cv::Point2f> imgPts;
  cv::projectPoints(board_pts, rvec, tvec, intrinsic, distort, imgPts,
                    cv::noArray(), 0);

  cv::Mat img_in = cv::imread(
      "/media/ramlab/SolidDisk/corners/pnp_cali/img/1635392634.922333.jpg");
  cv::eigen2cv(dist, distort);
  cv::Mat img_out;
  cv::undistort(img_in, img_out, intrinsic, distort);
  for (int i = 0; i < imgPts.size(); i++) {
    cv::circle(img_out, imgPts[i], 2, cv::Scalar(0, 0, 255));
  }
  for (int i = 0; i < corners.size(); i++) {
    cv::circle(img_out, corners[i], 2, cv::Scalar(0, 255, 0));
  }
  cv::imshow("image", img_out);
  cv::waitKey();
  transformation.setIdentity();
  transformation.topLeftCorner(3, 3) = rotm_ei;
  transformation.topRightCorner(3, 1) = t_ei;
  std::cout << "Transformation" << std::endl;
  std::cout << transformation << std::endl;
  return 0;
}
