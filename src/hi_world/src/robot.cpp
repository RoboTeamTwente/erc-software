#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace hi_world {

class RobotDriver : public webots_ros2_driver::PluginInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            [[maybe_unused]] std::unordered_map<std::string, std::string>
                &parameters) override {

    node_ = node;

    RCLCPP_INFO(rclcpp::get_logger("robot_driver"), "hi world");

    wheel_left_ = wb_robot_get_device("left wheel motor");
    wheel_right_ = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(wheel_left_, INFINITY);
    wb_motor_set_position(wheel_right_, INFINITY);

    wb_motor_set_velocity(wheel_left_, 0);
    wb_motor_set_velocity(wheel_right_, 0);

    auto cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](std::shared_ptr<geometry_msgs::msg::Twist const> msg) {
          RCLCPP_INFO(node_->get_logger(), "got twist %lf %lf", msg->linear.x,
                      msg->angular.z);
          cmd_vel_.linear = msg->linear;
          cmd_vel_.angular = msg->angular;
        });
  }

  void step() override {
    auto constexpr HALF_WHEEL_SEPARATION = 3;
    auto constexpr WHEEL_RADIUS = 3;

    auto fwd = cmd_vel_.linear.x;
    auto rot = cmd_vel_.angular.z;

    auto lvel = (fwd - rot * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS;
    auto rvel = (fwd + rot * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS;

    wb_motor_set_velocity(wheel_left_, lvel);
    wb_motor_set_velocity(wheel_right_, rvel);

    RCLCPP_INFO(node_->get_logger(), "step %lf %lf", cmd_vel_.linear.x,
                cmd_vel_.angular.z);
  }

private:
  webots_ros2_driver::WebotsNode *node_;
  WbDeviceTag wheel_left_, wheel_right_;
  geometry_msgs::msg::Twist cmd_vel_;
};

} // namespace hi_world

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hi_world::RobotDriver,
                       webots_ros2_driver::PluginInterface)
