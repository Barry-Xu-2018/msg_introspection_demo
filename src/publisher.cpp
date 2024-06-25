#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("test_publisher"), frame_idx_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = sensor_msgs::msg::Temperature();
        message.header.frame_id = std::to_string(frame_idx_);
        message.temperature = frame_idx_ + 0.1;
        message.variance = frame_idx_ + 0.2;

        this->publisher_->publish(message);
        frame_idx_++;
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
  uint32_t frame_idx_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}