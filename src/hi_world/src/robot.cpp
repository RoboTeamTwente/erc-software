#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace hi_world {

class RobotDriver : public webots_ros2_driver::PluginInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            [[maybe_unused]] std::unordered_map<std::string, std::string>
                &parameters) override {

    RCLCPP_INFO(rclcpp::get_logger("robot_driver"), "hi world");

    webots::Robot *robot = new webots::Robot();

    left_wheel_ = robot->getMotor("left wheel motor");
    right_wheel_ = robot->getMotor("right wheel motor");

    left_wheel_->setPosition(INFINITY);
    left_wheel_->setVelocity(0);

    right_wheel_->setPosition(INFINITY);
    right_wheel_->setVelocity(0);

    auto cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SensorDataQoS().reliable(),
        [this](std::shared_ptr<geometry_msgs::msg::Twist const> msg) {
          cmd_vel_.linear = msg->linear;
          cmd_vel_.angular = msg->angular;
        });
  }

  void step() override {
    auto constexpr HALF_WHEEL_SEPARATION = 3;
    auto constexpr WHEEL_RADIUS = 3;

    auto fwd = cmd_vel_.linear.x;
    auto rot = cmd_vel_.angular.z;

    left_wheel_->setVelocity((fwd - rot * HALF_WHEEL_SEPARATION) /
                             WHEEL_RADIUS);

    right_wheel_->setVelocity((fwd + rot * HALF_WHEEL_SEPARATION) /
                              WHEEL_RADIUS);
  }

private:
  webots::Motor *left_wheel_, *right_wheel_;
  geometry_msgs::msg::Twist cmd_vel_;
};

} // namespace hi_world

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hi_world::RobotDriver,
                       webots_ros2_driver::PluginInterface)
