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

namespace rtt_rover_driver {

class RobotDriver : public webots_ros2_driver::PluginInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &) override;

  void step() override;

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
PLUGINLIB_EXPORT_CLASS(rtt_rover_driver::RobotDriver,
                       webots_ros2_driver::PluginInterface)
