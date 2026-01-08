#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <vector>

using namespace std::chrono_literals;

static inline std::string trim_copy(std::string s) {
  auto not_space = [](unsigned char c){ return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

static inline bool is_integer(const std::string& s) {
  if (s.empty()) return false;
  return std::all_of(s.begin(), s.end(), [](unsigned char c){ return std::isdigit(c); });
}

static inline bool starts_with(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

// Prompts user once; returns either "0" or a URL / device index string
static std::string prompt_video_source() {
  std::cout
    << "\n=== ArUco Tracker ===\n"
    << "Select video source:\n"
    << "  [ENTER]  Use default USB camera (index 0)\n"
    << "  [0/1/2]  Use a different camera index\n"
    << "  [URL]    Use an IP camera / stream URL\n"
    << "\nExamples:\n"
    << "  http://192.168.0.152:8080/video   (IP Webcam)\n"
    << "  0\n"
    << "\nInput: ";

  std::string input;
  if (!std::getline(std::cin, input)) {
    // Non-interactive launch (stdin closed) -> fall back safely
    std::cout << "\n[Info] No stdin available; using default camera index 0.\n";
    return "0";
  }
  input = trim_copy(input);
  if (input.empty()) return "0";
  return input;
}


class ArucoTf2Node : public rclcpp::Node
{
public:
  ArucoTf2Node()
  : Node("aruco_tf2_node")
  {
    // Load calibration
    // NOTE: The yml file for the camera calibration is in "config/charuco_camera_params.yml". This is also mentioned in CMakeLists.txt
    // Also to get a new camera calibration, you need to generate it based on a Charuco Image Dataset (another CPP file)
    //TODO: Implement the Charuco Camera Calibration logic again. (no need for ROS2 impl, just standard C++)
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

    // --- NEW: choose video source interactively ---
    const std::string src = prompt_video_source();

    bool opened = false;
    if (is_integer(src)) {
      int idx = std::stoi(src);
      opened = cap_.open(idx, cv::CAP_ANY);
      RCLCPP_INFO(this->get_logger(), "Trying camera index %d", idx);
    } else {
      std::string url = src;

      // If user pasted "192.168.x.x:8080/video" without scheme, assume http://
      if (!starts_with(url, "http://") && !starts_with(url, "https://") && !starts_with(url, "rtsp://")) {
        url = "http://" + url;
      }

      // Try as-is first
      RCLCPP_INFO(this->get_logger(), "Trying stream URL: %s", url.c_str());
      opened = cap_.open(url, cv::CAP_ANY);

      // If they pasted just "...:8080" (base), IP Webcam often needs "/video"
      if (!opened) {
        std::string url2 = url;
        if (!starts_with(url2, "rtsp://") && url2.find("/video") == std::string::npos) {
          if (!url2.empty() && url2.back() == '/') url2.pop_back();
          url2 += "/video";
          RCLCPP_INFO(this->get_logger(), "Retrying stream URL: %s", url2.c_str());
          opened = cap_.open(url2, cv::CAP_ANY);
        }
      }
    }

    if (!opened) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open video source: '%s'", src.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Video source opened successfully.");


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      33ms, std::bind(&ArucoTf2Node::process_frame, this));
  }

private:
  void process_frame()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Frame grab failed");
      return;
    }
      //RCLCPP_INFO(this->get_logger(), "Frame grab success");

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

    // Note: Throttle the "No markers found" warning so your terminal doesn't scream in pain with a warning every 33ms
    if (ids.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,  // ms
        "No markers found"
      );
      return;
    } else {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
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

    // Note: If you put the next two lines at the top of the 'process_frame()' function, you will open the OpenCV window of your camera stream even if you don't detect ARUCO Markers.
    cv::imshow("aruco_view", frame);
    cv::waitKey(1);
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
