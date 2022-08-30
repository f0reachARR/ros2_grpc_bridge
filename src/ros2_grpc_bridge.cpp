#include <cstdio>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

class DeserializedGenericSubscription : public rclcpp::SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DeserializedGenericSubscription)

  template <typename AllocatorT = std::allocator<void>>
  DeserializedGenericSubscription(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic_name,
      const std::string &topic_type,
      const rclcpp::QoS &qos,
      std::function<void(std::shared_ptr<void>)> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options)
      : SubscriptionBase(
            node_base,
            *rclcpp::get_typesupport_handle(
                topic_type, "rosidl_typesupport_cpp",
                *rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp")),
            topic_name,
            options.template to_rcl_subscription_options<rclcpp::SerializedMessage>(qos),
            false),
        deserialized_callback_(callback)
  {
    if (options.event_callbacks.deadline_callback)
    {
      this->add_event_handler(
          options.event_callbacks.deadline_callback,
          RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
    }
    if (options.event_callbacks.liveliness_callback)
    {
      this->add_event_handler(
          options.event_callbacks.liveliness_callback,
          RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
    }
    if (options.event_callbacks.incompatible_qos_callback)
    {
      this->add_event_handler(
          options.event_callbacks.incompatible_qos_callback,
          RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
    }
    else if (options.use_default_callbacks)
    {
      // Register default callback when not specified
      try
      {
        this->add_event_handler(
            [this](rclcpp::QOSRequestedIncompatibleQoSInfo &info)
            {
              this->default_incompatible_qos_callback(info);
            },
            RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
      }
      catch (rclcpp::UnsupportedEventTypeException & /*exc*/)
      {
        // pass
      }
    }
    if (options.event_callbacks.message_lost_callback)
    {
      this->add_event_handler(
          options.event_callbacks.message_lost_callback,
          RCL_SUBSCRIPTION_MESSAGE_LOST);
    }

    using rosidl_typesupport_introspection_cpp::MessageMembers;

    auto typesupport_identifier = "rosidl_typesupport_introspection_cpp";
    auto intro_lib = rclcpp::get_typesupport_library(topic_type, typesupport_identifier);
    auto typesupport = rclcpp::get_typesupport_handle(topic_type, typesupport_identifier, *intro_lib);

    members_ = static_cast<const MessageMembers *>(typesupport->data);
  }

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override
  {
    return std::make_shared<rclcpp::SerializedMessage>(0);
  }

  std::shared_ptr<void> create_message() override
  {
    using rosidl_runtime_cpp::MessageInitialization;

    auto buffer = calloc(1, members_->size_of_);
    members_->init_function(buffer, MessageInitialization::ZERO);

    auto deleter = [&](void *buffer)
    {
      members_->fini_function(buffer);
      free(buffer);
    };
    return std::shared_ptr<void>(buffer, deleter);
  }

  bool is_serialized() const
  {
    std::cout << "is_serialized" << std::endl;
    return false;
  }

  bool can_loan_messages()
  {
    return false;
  }

  void handle_message(
      std::shared_ptr<void> &message, const rclcpp::MessageInfo &message_info) override
  {
    (void)message_info;
    deserialized_callback_(message);
  }

  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override
  {
    throw rclcpp::exceptions::UnimplementedError(
        "handle_loaned_message is not implemented for GenericSubscription");
  }

  void return_message(std::shared_ptr<void> &message) override
  {
    message.reset();
  }

  void return_serialized_message(
      std::shared_ptr<rclcpp::SerializedMessage> &message) override
  {
    message.reset();
  }

private:
  std::function<void(std::shared_ptr<void>)> deserialized_callback_;
  const rosidl_typesupport_introspection_cpp::MessageMembers *members_;
};

template <typename AllocatorT = std::allocator<void>>
std::shared_ptr<DeserializedGenericSubscription> create_deserialized_generic_subscription(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string &topic_name,
    const std::string &topic_type,
    const rclcpp::QoS &qos,
    std::function<void(std::shared_ptr<void>)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options = (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()))
{
  auto subscription = std::make_shared<DeserializedGenericSubscription>(
      topics_interface->get_node_base_interface(),
      topic_name,
      topic_type,
      qos,
      callback,
      options);

  topics_interface->add_subscription(subscription, options.callback_group);

  return subscription;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("grpc_bridge");
  auto topic_type = "sensor_msgs/msg/LaserScan";

  auto ts_lib = rclcpp::get_typesupport_library(
      topic_type, "rosidl_typesupport_cpp");
  auto sub = create_deserialized_generic_subscription(
      node->get_node_topics_interface(), "/lrf_head_scan",
      topic_type, rclcpp::QoS(10),
      [&](std::shared_ptr<void> message)
      {
        (void)message;
        auto test = std::static_pointer_cast<sensor_msgs::msg::LaserScan>(message);
        std::cout << test->ranges.size() << std::endl;
        // RMW deserialize -> done
        // Protobuf serialize
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
