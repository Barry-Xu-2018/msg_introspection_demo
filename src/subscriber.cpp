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
    const std::string topic_type = "sensor_msgs/msg/Temperature";

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
      if (std::strcmp(members_->members_[i].name_, "temperature") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64) {
           RCLCPP_INFO(this->get_logger(),"temperature: %f", *(double *)(deserialized_message.get() + members_->members_[i].offset_));
        }
      }

      if (std::strcmp(members_->members_[i].name_, "variance") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64) {
          RCLCPP_INFO(this->get_logger(),"variance: %f", *(double *)(deserialized_message.get() + members_->members_[i].offset_));
        }
      }

      if (std::strcmp(members_->members_[i].name_, "header") == 0) {
        if (members_->members_[i].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          auto sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(members_->members_[i].members_->data);
          auto header_offset = members_->members_[i].offset_;
          for(size_t j = 0; j < sub_members->member_count_; j++) {
            if (std::strcmp(sub_members->members_[j].name_,"frame_id") == 0) {
              if (sub_members->members_[j].type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING) {
                std::string *str = (std::string *)(deserialized_message.get() + header_offset + sub_members->members_[j].offset_);
                RCLCPP_INFO(this->get_logger(),"frame_id: %s", str->c_str());
              }
            }
          }
        }
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