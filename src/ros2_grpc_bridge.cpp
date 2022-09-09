#include <cstdio>
#include <grpcpp/support/status.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_protobuf/message_type_support.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <grpc/grpc.h>

#include <grpc++/generic/async_generic_service.h>
#include <grpc++/grpc++.h>
#include <grpc++/server_builder.h>

#include <grpc++/impl/codegen/config_protobuf.h>

#include "ros/cppmsg_constructor.hpp"
#include "ros/dynamic_subscription.hpp"

#include "grpc/ros_descriptor.hpp"
#include "grpc/ros_reflection.hpp"
class RosSubscribeReactor : public grpc::ServerGenericBidiReactor {
public:
  explicit RosSubscribeReactor(std::string topic) {
    auto topic_type = "sensor_msgs/msg/LaserScan";

    auto proto_identifier = "rosidl_typesupport_protobuf_cpp";
    auto proto_ts_lib =
        rclcpp::get_typesupport_library(topic_type, proto_identifier);
    auto proto_ts_handle = rclcpp::get_typesupport_handle(
        topic_type, proto_identifier, *proto_ts_lib);
    typesupport_pb_ = static_cast<
        const rosidl_typesupport_protobuf::message_type_support_t *>(
        proto_ts_handle->data);

    sensor_msgs::msg::LaserScan msg;
    std::string pbout;
    typesupport_pb_->serialize(&msg, pbout);

    slice = grpc::Slice(pbout);
    buf = grpc::ByteBuffer(&slice, 1);

    StartWrite(&buf);
  }

  void OnWriteDone(bool ok) override { Finish(grpc::Status::OK); }

  void OnDone() override { delete this; }

private:
  std::shared_ptr<DynamicSubscription> sub_;
  const rosidl_typesupport_protobuf::message_type_support_t *typesupport_pb_;
  grpc::Slice slice;
  grpc::ByteBuffer buf;
};

class UnknownReactor : public grpc::ServerGenericBidiReactor {
public:
  explicit UnknownReactor() {
    this->Finish(grpc::Status(grpc::StatusCode::UNIMPLEMENTED, ""));
  }
  void OnDone() override { delete this; }
};

class RosService : public grpc::CallbackGenericService {
public:
  grpc::ServerGenericBidiReactor *
  CreateReactor(grpc::GenericCallbackServerContext *ctx) override {
    std::cout << ctx->method() << std::endl;
    if (ctx->method() == "/lrf_head_scan.Topic/Subscribe") {
      return new RosSubscribeReactor("lrf_head_scan");
    }
    return new UnknownReactor;
  }
};

void register_service() {}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  google::protobuf::DescriptorPool pool;
  pool.internal_set_underlay(
      google::protobuf::DescriptorPool::generated_pool());
  create_topic_service(pool, "lrf_head_scan", "sensor_msgs/msg/LaserScan");

  grpc::ServerBuilder builder;
  RosService service;
  ProtoServerReflection reflection(pool);
  builder.AddListeningPort("0.0.0.0:8080", grpc::InsecureServerCredentials());
  builder.RegisterService(&reflection);
  builder.RegisterCallbackGenericService(&service);

  auto server = builder.BuildAndStart();

  server->Wait();
  // rclcpp::init(argc, argv);

  // auto node = std::make_shared<rclcpp::Node>("grpc_bridge");

  // auto sub = create_dynamic_subscription(
  //     node->get_node_topics_interface(), "/lrf_head_scan",
  //     topic_type, rclcpp::QoS(10),
  //     [&](std::shared_ptr<void> message)
  //     {
  //       (void)message;
  //       auto test =
  //       std::static_pointer_cast<sensor_msgs::msg::LaserScan>(message);
  //       std::string pbout;
  //       proto_ts_support->serialize(message.get(), pbout);
  //       std::cout << pbout << std::endl;
  //       // RMW deserialize -> done
  //       // Protobuf serialize
  //     });

  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
