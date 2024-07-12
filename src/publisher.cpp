// Copyright 2024 Sony Group Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "msg_introspection_demo/msg/my_message.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("test_publisher"), frame_idx_(0)
  {
    publisher_ = this->create_publisher<msg_introspection_demo::msg::MyMessage>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = msg_introspection_demo::msg::MyMessage();
        message.header.frame_id = "frame " + std::to_string(frame_idx_);
        message.index = frame_idx_;
        message.name = "hello " + std::to_string(frame_idx_);
        message.bounded_array = {1000 + frame_idx_, 1300 + frame_idx_, 1500 + frame_idx_};
        message.fixed_gap = 0x5555;
        message.unbounded_array = {3000 + frame_idx_, 3300 + frame_idx_, 3500 + frame_idx_, 3700 + frame_idx_};
        message.end = (int64_t)frame_idx_ * -1;
        this->publisher_->publish(message);
        frame_idx_++;
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<msg_introspection_demo::msg::MyMessage>::SharedPtr publisher_;
  uint32_t frame_idx_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}