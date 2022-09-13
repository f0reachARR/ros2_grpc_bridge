#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

class MessageConstructor {
private:
  static constexpr auto INTROSPECTION_IDENTIFIER =
      "rosidl_typesupport_introspection_cpp";

public:
  explicit MessageConstructor(const std::string data_type) {
    using rosidl_typesupport_introspection_cpp::MessageMembers;

    typesupport_lib_ =
        rclcpp::get_typesupport_library(data_type, INTROSPECTION_IDENTIFIER);
    auto typesupport = rclcpp::get_typesupport_handle(
        data_type, INTROSPECTION_IDENTIFIER, *typesupport_lib_);

    members_ = static_cast<const MessageMembers *>(typesupport->data);
  }

  std::shared_ptr<void> create_message() {
    using rosidl_runtime_cpp::MessageInitialization;

    auto buffer = calloc(1, members_->size_of_);
    members_->init_function(buffer, MessageInitialization::ZERO);

    auto deleter = [&](void *buffer) {
      members_->fini_function(buffer);
      free(buffer);
    };
    return std::shared_ptr<void>(buffer, deleter);
  }

private:
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib_;
  const rosidl_typesupport_introspection_cpp::MessageMembers *members_;
};