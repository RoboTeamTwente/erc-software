#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace simulation {

class RobotDriver : public webots_ros2_driver::PluginInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &) override {

    node_ = node;

    RCLCPP_INFO(rclcpp::get_logger("robot_driver"), "hi world");

    gps_ = wb_robot_get_device("gps");
    cam_ = wb_robot_get_device("camera");

    wb_camera_enable(cam_, 15 /* FPS */);

    wheel_bl_ = wb_robot_get_device("left back wheel motor");
    wheel_ml_ = wb_robot_get_device("left middle wheel motor");
    wheel_fl_ = wb_robot_get_device("left front wheel motor");
    wheel_br_ = wb_robot_get_device("right back wheel motor");
    wheel_mr_ = wb_robot_get_device("right middle wheel motor");
    wheel_fr_ = wb_robot_get_device("right front wheel motor");

    wb_motor_set_position(wheel_bl_, INFINITY);
    wb_motor_set_position(wheel_ml_, INFINITY);
    wb_motor_set_position(wheel_fl_, INFINITY);
    wb_motor_set_position(wheel_br_, INFINITY);
    wb_motor_set_position(wheel_mr_, INFINITY);
    wb_motor_set_position(wheel_fr_, INFINITY);

    wb_motor_set_velocity(wheel_bl_, 0);
    wb_motor_set_velocity(wheel_ml_, 0);
    wb_motor_set_velocity(wheel_fl_, 0);
    wb_motor_set_velocity(wheel_br_, 0);
    wb_motor_set_velocity(wheel_mr_, 0);
    wb_motor_set_velocity(wheel_fr_, 0);

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](std::shared_ptr<geometry_msgs::msg::Twist const> msg) {
          RCLCPP_INFO(node_->get_logger(), "got twist %lf %lf", msg->linear.x,
                      msg->angular.z);
          cmd_vel_.linear = msg->linear;
          cmd_vel_.angular = msg->angular;
        });

    RCLCPP_INFO(node_->get_logger(), "sub created %p",
                (void *)cmd_vel_sub_.get());
  }

  void step() override {
    auto constexpr HALF_WHEEL_SEPARATION = 3;
    auto constexpr WHEEL_RADIUS = 3;

    auto fwd = cmd_vel_.linear.x;
    auto rot = cmd_vel_.angular.z;

    auto lvel = (fwd - rot * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS;
    auto rvel = (fwd + rot * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS;

    wb_motor_set_velocity(wheel_bl_, lvel);
    wb_motor_set_velocity(wheel_ml_, lvel);
    wb_motor_set_velocity(wheel_fl_, lvel);
    wb_motor_set_velocity(wheel_br_, rvel);
    wb_motor_set_velocity(wheel_mr_, rvel);
    wb_motor_set_velocity(wheel_fr_, rvel);

    // RCLCPP_INFO(node_->get_logger(), "step %lf %lf", cmd_vel_.linear.x,
    //             cmd_vel_.angular.z);
  }

private:
  webots_ros2_driver::WebotsNode *node_;
  WbDeviceTag gps_, cam_;
  WbDeviceTag wheel_bl_, wheel_br_;
  WbDeviceTag wheel_ml_, wheel_mr_;
  WbDeviceTag wheel_fl_, wheel_fr_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
  geometry_msgs::msg::Twist cmd_vel_;
};

} // namespace hi_world

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(simulation::RobotDriver,
                       webots_ros2_driver::PluginInterface)
