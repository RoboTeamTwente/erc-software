extern "C" {
#include <control.h>
}

#include "driver.hpp"
#include <cmath>
#include <cstddef>
#include <rclcpp/logger.hpp>
#include <webots/motor.h>
#include <webots/position_sensor.h>

void rtt_rover_driver::RobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &) {

  node_ = node;

  RCLCPP_INFO(node_->get_logger(), "starting initialization");

  control_initialize();

  // same as the world's sample rate
  auto constexpr SAMPLE_RATE = 32;

  gps_ = wb_robot_get_device("gps");
  cam_ = wb_robot_get_device("camera");

  wb_camera_enable(cam_, SAMPLE_RATE);

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
    wb_position_sensor_enable(steering_encoders_[i], SAMPLE_RATE);
  }

  for (auto &w : motors_) {
    wb_motor_set_position(w, INFINITY);
    wb_motor_set_velocity(w, 0);
  }

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&rtt_rover_driver::RobotDriver::process_command, this,
                std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "initialization done");
}

void rtt_rover_driver::RobotDriver::process_command(
    std::shared_ptr<geometry_msgs::msg::Twist const> msg) {
  RCLCPP_INFO(node_->get_logger(), "got twist %lf %lf", msg->linear.x,
              msg->angular.z);

  rtU.alpha = msg->angular.z;
  rtU.dist2goal = msg->linear.x;
  rtU.R = std::abs(rtU.dist2goal / rtU.alpha * M_PI / 180);
}

void rtt_rover_driver::RobotDriver::step() {
  for (size_t i = 0; i < motors_.size(); i++) {
    rtU.actspeed[i] = wb_motor_get_velocity(motors_[i]);
  }

  for (size_t i = 0; i < steering_.size(); i++) {
    rtU.actspeed[i] = wb_position_sensor_get_value(steering_encoders_[i]);
  }

  control_step();

  RCLCPP_INFO(node_->get_logger(),
              "\n"
              "control inputs: R=%lf alpha=%lf dist=%lf\n"
              "control outputs: motors=[%lf %lf %lf %lf %lf %lf]\n"
              "control outputs: steering=[%lf %lf %lf %lf]",
              rtU.R, rtU.alpha, rtU.dist2goal, //
              rtY.controlb[0], rtY.controlb[1], rtY.controlb[2],
              rtY.controlb[3], rtY.controlb[4], rtY.controlb[5], //
              rtY.desang[0], rtY.desang[1], rtY.desang[2], rtY.desang[3]);

  for (size_t i = 0; i < motors_.size(); i++) {
    // controlb is on scale 0V-24V
    // desspeed is for debugging
    assert(!std::isnan(rtY.controlb[i]));
    wb_motor_set_velocity(motors_[i], rtY.controlb[i]);
  }

  for (size_t i = 0; i < steering_.size(); i++) {
    // desang is for debugging as well,
    // but I don't wanna parse PWM signals
    assert(!std::isnan(rtY.desang[i]));
    wb_motor_set_position(steering_[i], rtY.desang[i] * M_PI / 180);
  }
}
