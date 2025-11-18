#include "driver.hpp"

#include <rclcpp/logger.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Scalar.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

extern "C" {
#include <control.h>
}

void rtt_rover_driver::RobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &) {

  node_ = node;

  RCLCPP_INFO(node_->get_logger(), "starting initialization");

  control_initialize();

  sample_rate_ = wb_robot_get_basic_time_step();
  RCLCPP_INFO(node_->get_logger(), "sample rate %lf", sample_rate_);

  gps_ = wb_robot_get_device("gps");
  cam_ = wb_robot_get_device("camera");
  imu_ = wb_robot_get_device("inertial unit");

  wb_camera_enable(cam_, sample_rate_);
  wb_gps_enable(gps_, sample_rate_);
  wb_inertial_unit_enable(imu_, sample_rate_);

  assert(wb_gps_get_coordinate_system(gps_) == WB_GPS_LOCAL_COORDINATE);

  motors_[0] = wb_robot_get_device("left front wheel motor");
  motors_[1] = wb_robot_get_device("left middle wheel motor");
  motors_[2] = wb_robot_get_device("left back wheel motor");
  motors_[3] = wb_robot_get_device("right front wheel motor");
  motors_[4] = wb_robot_get_device("right middle wheel motor");
  motors_[5] = wb_robot_get_device("right back wheel motor");

  steering_[0] = wb_robot_get_device("left front steering motor");
  steering_[1] = wb_robot_get_device("left back steering motor");
  steering_[2] = wb_robot_get_device("right front steering motor");
  steering_[3] = wb_robot_get_device("right back steering motor");

  for (size_t i = 0; i < steering_.size(); i++) {
    steering_encoders_[i] = wb_motor_get_position_sensor(steering_[i]);
    wb_position_sensor_enable(steering_encoders_[i], sample_rate_);
  }

  for (auto &w : motors_) {
    wb_motor_set_position(w, std::numeric_limits<double>::infinity());
    wb_motor_set_velocity(w, 0);
  }

  goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::SensorDataQoS().reliable(),
      [this](std::shared_ptr<geometry_msgs::msg::PoseStamped const> msg) {
        RCLCPP_INFO(node_->get_logger(), "got goal %lf %lf %lf",
                    msg->pose.position.x, msg->pose.position.y,
                    msg->pose.position.z);

        goal_pose_ = msg->pose;
      });

  RCLCPP_INFO(node_->get_logger(), "initialization done");
}

void rtt_rover_driver::RobotDriver::step() {
  // wait for the first sample
  if (wb_robot_get_time() * 1000 < sample_rate_)
    return;

  for (size_t i = 0; i < motors_.size(); i++) {
    rtU.actspeed[i] = wb_motor_get_velocity(motors_[i]);
  }

  for (size_t i = 0; i < steering_.size(); i++) {
    rtU.actang[i] = wb_position_sensor_get_value(steering_encoders_[i]);
  }

  auto position_raw = wb_gps_get_values(gps_);
  auto pose_raw = wb_inertial_unit_get_quaternion(imu_);

  tf2::Vector3 position{position_raw[0], position_raw[1], position_raw[2]};
  tf2::Quaternion orientation{pose_raw[0], pose_raw[1], pose_raw[2],
                              pose_raw[3]};

  tf2::Vector3 goal_position;
  tf2::Quaternion goal_orientation;
  tf2::convert(goal_pose_.position, goal_position);
  tf2::convert(goal_pose_.orientation, goal_orientation);

  tf2::Matrix3x3 relative_goal_orientation{
      (orientation * goal_orientation.inverse()).normalized()};

  tf2Scalar roll, pitch, yaw;
  relative_goal_orientation.getRPY(roll, pitch, yaw);

  tf2Scalar gr, gp, gy;
  tf2::Matrix3x3{goal_orientation}.getRPY(gr, gp, gy);

  tf2Scalar pr, pp, py;
  tf2::Matrix3x3{orientation}.getRPY(pr, pp, py);

  rtU.alpha = yaw * 180 / M_PI;
  rtU.dist2goal = position.distance(goal_position);
  rtU.R = 1; // std::abs(rtU.dist2goal / rtU.alpha * M_PI / 180);

  control_step();

  RCLCPP_INFO(node_->get_logger(),
              "\n"
              "control inputs: R=%lf alpha=%lf dist=%lf\n"
              "control outputs: motors=[%lf %lf %lf %lf %lf %lf]\n"
              "speed: [%lf %lf %lf %lf %lf %lf] [%lf %lf %lf %lf %lf %lf]\n"
              "control outputs: steering=[%lf %lf %lf %lf]\n"
              "GPS: %lf %lf %lf "
              "IMU: %lf %lf %lf",
              rtU.R, rtU.alpha, rtU.dist2goal, //
              rtY.controlb[0], rtY.controlb[1], rtY.controlb[2],
              rtY.controlb[3], rtY.controlb[4], rtY.controlb[5], //
              rtU.actspeed[0], rtU.actspeed[1], rtU.actspeed[2],
              rtU.actspeed[3], rtU.actspeed[4], rtU.actspeed[5], //
              rtY.desspeed[0], rtY.desspeed[1], rtY.desspeed[2],
              rtY.desspeed[3], rtY.desspeed[4], rtY.desspeed[5],          //
              rtY.desang[0], rtY.desang[1], rtY.desang[2], rtY.desang[3], //
              position[0], position[1], position[2],                      //
              roll, pitch, yaw);

  for (size_t i = 0; i < motors_.size(); i++) {
    // controlb is on scale 0V-24V
    // desspeed is for debugging
    if (!std::isnan(rtY.controlb[i])) {
      wb_motor_set_velocity(motors_[i], rtY.controlb[i]);
    }
  }

  for (size_t i = 0; i < steering_.size(); i++) {
    // desang is for debugging as well,
    // but I don't wanna parse PWM signals
    if (!std::isnan(rtY.desang[i])) {
      wb_motor_set_position(steering_[i], rtY.desang[i] * M_PI / 180);
    }
  }
}
