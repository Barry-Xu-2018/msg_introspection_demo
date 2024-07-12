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

#include <functional>
#include <memory>
#include "rmw/rmw.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
  : Node("test_subscriber")
  {
    const std::string topic_type = "msg_introspection_demo/msg/MyMessage";

    ts_lib_introspection_ = rclcpp::get_typesupport_library(
      topic_type, "rosidl_typesupport_introspection_cpp");
    type_support_introspection_ = rclcpp::get_message_typesupport_handle(
      topic_type, "rosidl_typesupport_introspection_cpp", *ts_lib_introspection_);

    ts_lib_ = rclcpp::get_typesupport_library(
      topic_type, "rosidl_typesupport_cpp");
    type_support_ = rclcpp::get_message_typesupport_handle(
      topic_type, "rosidl_typesupport_cpp", *ts_lib_);

    members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      type_support_introspection_->data);

    subscription_ = this->create_generic_subscription(
      "topic",
      topic_type,
      10,
      std::bind(&Subscriber::topic_callback, this, _1, _2));
  }

private:
  void topic_callback(
    std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo &)
  {
    RCLCPP_INFO(this->get_logger(), "+++ topic_callback");

    // Allocating memory required for deserialization
    auto deserialized_message = std::shared_ptr<uint8_t[]>(
      new uint8_t[members_->size_of_],
      [fini_function = members_->fini_function](uint8_t * msg) {
      fini_function(msg);
      delete[] msg;
    });

    // Initialize the allocated memory
    members_->init_function(
      deserialized_message.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);

    // Deserialize the received message
    rmw_ret_t ret =
      rmw_deserialize(
        &message->get_rcl_serialized_message(),
        type_support_,
        deserialized_message.get());
    if (ret != RMW_RET_OK) {
      RCLCPP_INFO(this->get_logger(), "Deserialization failed: %d", ret);
      return;
    }

    // In this example, the offset is fixed, so it can be calculated in advance and doesn't need to
    // be placed in this callback.
    for (size_t i = 0; i < members_->member_count_; i++) {
      // std::string
      if (std::strcmp(members_->members_[i].name_, "name") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
           RCLCPP_INFO(this->get_logger(),"name: %s", (*(std::string *)(deserialized_message.get() + members_->members_[i].offset_)).c_str());
        }
        continue;
      }

      // uint64
      if (std::strcmp(members_->members_[i].name_, "index") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64) {
           RCLCPP_INFO(this->get_logger(),"index: %lu", *(uint64_t *)(deserialized_message.get() + members_->members_[i].offset_));
        }
        continue;
      }

      // submember std::string
      if (std::strcmp(members_->members_[i].name_, "header") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          auto sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(members_->members_[i].members_->data);
          auto header_offset = members_->members_[i].offset_;
          for(size_t j = 0; j < sub_members->member_count_; j++) {
            if (std::strcmp(sub_members->members_[j].name_,"frame_id") == 0) {
              if (sub_members->members_[j].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
                std::string *str = (std::string *)(deserialized_message.get() + header_offset + sub_members->members_[j].offset_);
                RCLCPP_INFO(this->get_logger(),"header.frame_id: %s", str->c_str());
              }
            }
          }
        }
        continue;
      }

      // bounded array uint32[3]
      if (std::strcmp(members_->members_[i].name_, "bounded_array") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) {
          // For bounded array, array_size_ is the size of array
          if (members_->members_[i].is_array_ && members_->members_[i].array_size_ == 3) {
            auto bounded_array = *(std::array<uint32_t, 3> *)(deserialized_message.get() + members_->members_[i].offset_);
            RCLCPP_INFO(this->get_logger(),"bounded_array: [%u, %u, %u]", bounded_array[0], bounded_array[1], bounded_array[2]);
          }
        }
        continue;
      }

      // unbounded array uint32[]
      if (std::strcmp(members_->members_[i].name_, "unbounded_array") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) {
          // For unbounded array, array_size_ is 0
          if (members_->members_[i].is_array_ && members_->members_[i].array_size_ == 0) {
            auto unbounded_array = *(std::vector<uint32_t> *)(deserialized_message.get() + members_->members_[i].offset_);
            RCLCPP_INFO(this->get_logger(),"unbounded_array %lu: [%u, %u, %u, %u]",
              unbounded_array.size(), unbounded_array[0], unbounded_array[1], unbounded_array[2], unbounded_array[3]);
          }
        }
        continue;
      }

      // int64
      if (std::strcmp(members_->members_[i].name_, "end") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64) {
          // This item is after a variable array in msg, you can still retrieve data using an offset.
          RCLCPP_INFO(this->get_logger(),"end: %ld", *(int64_t *)(deserialized_message.get() + members_->members_[i].offset_));
        }
        continue;
      }
    }
    printf("\n\n");
  }
  rclcpp::GenericSubscription::SharedPtr subscription_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * type_support_introspection_;
  const rosidl_message_type_support_t * type_support_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}