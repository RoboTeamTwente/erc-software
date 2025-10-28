#include "sensor_msgs/msg/joint_state.hpp"
#include <cstdio>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

using namespace std::chrono_literals;

class HiWorld : public rclcpp::Node {
public:
  HiWorld() : Node("hi_world") {
    joint_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    timer_ = create_wall_timer(33ms, [this]() {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = get_clock()->now();
      msg.name = {"swivel", "tilt", "periscope"}; // joints being published
      msg.position = {counter_++, counter_++, counter_++}; // joints values
      joint_pub_->publish(msg);
      // RCLCPP_INFO(get_logger(), "Published: %lf %lf %lf", msg.position[0],
      //             msg.position[1], msg.position[2]);
    });
  }

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;

  double counter_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HiWorld>());
  rclcpp::shutdown();
  return 0;
}
