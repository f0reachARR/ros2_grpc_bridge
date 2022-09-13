#pragma once

#include "cppmsg_constructor.hpp"
#include <rclcpp/rclcpp.hpp>

class DynamicSubscription : public rclcpp::SubscriptionBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DynamicSubscription)

  template <typename AllocatorT = std::allocator<void>>
  DynamicSubscription(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic_name, const std::string &topic_type,
      const rosidl_message_type_support_t *typesupport, const rclcpp::QoS &qos,
      std::function<void(std::shared_ptr<void>)> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options)
      : SubscriptionBase(node_base, *typesupport, topic_name,
                         options.template to_rcl_subscription_options<
                             rclcpp::SerializedMessage>(qos),
                         false),
        deserialized_callback_(callback), dyn_construct_(topic_type) {
    if (options.event_callbacks.deadline_callback) {
      this->add_event_handler(options.event_callbacks.deadline_callback,
                              RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
    }
    if (options.event_callbacks.liveliness_callback) {
      this->add_event_handler(options.event_callbacks.liveliness_callback,
                              RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
    }
    if (options.event_callbacks.incompatible_qos_callback) {
      this->add_event_handler(options.event_callbacks.incompatible_qos_callback,
                              RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
    } else if (options.use_default_callbacks) {
      // Register default callback when not specified
      try {
        this->add_event_handler(
            [this](rclcpp::QOSRequestedIncompatibleQoSInfo &info) {
              this->default_incompatible_qos_callback(info);
            },
            RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
      } catch (rclcpp::UnsupportedEventTypeException & /*exc*/) {
        // pass
      }
    }
    if (options.event_callbacks.message_lost_callback) {
      this->add_event_handler(options.event_callbacks.message_lost_callback,
                              RCL_SUBSCRIPTION_MESSAGE_LOST);
    }
  }

  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override {
    return std::make_shared<rclcpp::SerializedMessage>(0);
  }

  std::shared_ptr<void> create_message() override {
    return dyn_construct_.create_message();
  }

  bool can_loan_messages() { return false; }

  void handle_message(std::shared_ptr<void> &message,
                      const rclcpp::MessageInfo &message_info) override {
    (void)message_info;
    deserialized_callback_(message);
  }

  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override {
    throw rclcpp::exceptions::UnimplementedError(
        "handle_loaned_message is not implemented for GenericSubscription");
  }

  void return_message(std::shared_ptr<void> &message) override {
    message.reset();
  }

  void return_serialized_message(
      std::shared_ptr<rclcpp::SerializedMessage> &message) override {
    message.reset();
  }

private:
  static const rosidl_message_type_support_t *
  get_typesupport(const std::string &type) {
    auto ts_lib =
        rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    auto handle =
        rclcpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", *ts_lib);
    return handle;
  }

private:
  std::function<void(std::shared_ptr<void>)> deserialized_callback_;
  MessageConstructor dyn_construct_;
};

template <typename AllocatorT = std::allocator<void>>
std::shared_ptr<DynamicSubscription> create_dynamic_subscription(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string &topic_name, const std::string &topic_type,
    const rclcpp::QoS &qos, std::function<void(std::shared_ptr<void>)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options =
        (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>())) {

  auto ts_lib =
      rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  auto handle = rclcpp::get_typesupport_handle(
      topic_type, "rosidl_typesupport_cpp", *ts_lib);
  auto subscription = std::make_shared<DynamicSubscription>(
      topics_interface->get_node_base_interface(), topic_name, topic_type,
      handle, qos, callback, options);

  topics_interface->add_subscription(subscription, options.callback_group);

  return subscription;
}
