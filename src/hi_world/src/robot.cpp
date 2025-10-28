#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cstdio>
#include <memory>
#include <rcl/timer.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/string.hpp>

using std::chrono::seconds;

class HiWorld : public rclcpp::Node {
public:
  HiWorld() : Node("hi_world") {
    publisher_ = create_publisher<std_msgs::msg::String>("hi_world", 10);
    timer_ = create_wall_timer(seconds(1), [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "hi world";
      publisher_->publish(msg);
      RCLCPP_INFO(get_logger(), "Published: %s", msg.data.c_str());
    });
  }

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HiWorld>());
  rclcpp::shutdown();
  return 0;
}
