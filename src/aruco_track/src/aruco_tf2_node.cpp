#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class ArucoTf2Node : public rclcpp::Node
{
public:
  ArucoTf2Node()
  : Node("aruco_tf2_node")
  {
    // Load calibration
    std::string calib_path =
    ament_index_cpp::get_package_share_directory("aruco_track") +
    "/charuco_camera_params.yml";
    cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();

    if (camera_matrix_.empty()) {
        throw std::runtime_error("Failed to load camera calibration");
    }


    marker_length_ = 0.05;  // meters

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    detector_params_ = cv::aruco::DetectorParameters::create();

    cap_.open("http://145.126.47.250:8080/video", cv::CAP_FFMPEG);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open video stream");
      rclcpp::shutdown();
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      33ms, std::bind(&ArucoTf2Node::process_frame, this));
  }

private:
  void process_frame()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "Frame grab failed");
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

    if (ids.empty()) {
      return;
    }

    cv::aruco::estimatePoseSingleMarkers(
      corners,
      marker_length_,
      camera_matrix_,
      dist_coeffs_,
      rvecs,
      tvecs
    );

    for (size_t i = 0; i < ids.size(); ++i) {
      publish_transform(ids[i], rvecs[i], tvecs[i]);
    }
  }

  void publish_transform(int id, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera_frame";               // IMPORTANT
    t.child_frame_id = "aruco_" + std::to_string(id);

    t.transform.translation.x = tvec[0];
    t.transform.translation.y = tvec[1];
    t.transform.translation.z = tvec[2];

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    tf2::Matrix3x3 tf_R(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );

    tf2::Quaternion q;
    tf_R.getRotation(q);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  // Members
  cv::VideoCapture cap_;
  cv::Mat camera_matrix_, dist_coeffs_;
  double marker_length_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ArucoTf2Node>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("aruco_tf2_node"), e.what());
  }
  rclcpp::shutdown();
  return 0;
}
