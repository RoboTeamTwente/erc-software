#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace rtt_rover_driver {

class RobotDriver : public webots_ros2_driver::PluginInterface {
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &) override;

  void step() override;

private:
  webots_ros2_driver::WebotsNode *node_;

  WbDeviceTag gps_, cam_,imu_;
  std::array<WbDeviceTag, 6> motors_;
  std::array<WbDeviceTag, 4> steering_, steering_encoders_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>
      goal_pose_sub_;
  geometry_msgs::msg::Pose goal_pose_;
};

} // namespace rtt_rover_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rtt_rover_driver::RobotDriver,
                       webots_ros2_driver::PluginInterface)
