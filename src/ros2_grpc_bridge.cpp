#include <cstdio>
#include <functional>
#include <grpcpp/support/byte_buffer.h>
#include <grpcpp/support/status.h>
#include <iostream>

#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_protobuf/message_type_support.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <grpc/grpc.h>

#include <grpc++/generic/async_generic_service.h>
#include <grpc++/grpc++.h>
#include <grpc++/server_builder.h>

#include <grpc++/impl/codegen/config_protobuf.h>
#include <thread>

#include "ros/cppmsg_constructor.hpp"
#include "ros/dynamic_subscription.hpp"

#include "grpc/ros_descriptor.hpp"
#include "grpc/ros_reflection.hpp"
class RosSubscribeReactor : public grpc::ServerGenericBidiReactor {
public:
  explicit RosSubscribeReactor(
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_base,
      std::string topic) {
    auto topic_type = "sensor_msgs/msg/LaserScan";
    sub_ = create_dynamic_subscription(
        node_base, topic, topic_type, rclcpp::QoS(10),
        std::bind(&RosSubscribeReactor::SerializeAndWrite, this,
                  std::placeholders::_1));
  }

  void SerializeAndWrite(std::shared_ptr<void> buffer) {
    auto topic_type = "sensor_msgs/msg/LaserScan";
    auto proto_identifier = "rosidl_typesupport_protobuf_cpp";

    auto proto_ts_lib =
        rclcpp::get_typesupport_library(topic_type, proto_identifier);
    auto proto_ts_handle = rclcpp::get_typesupport_handle(
        topic_type, proto_identifier, *proto_ts_lib);
    typesupport_pb_ = static_cast<
        const rosidl_typesupport_protobuf::message_type_support_t *>(
        proto_ts_handle->data);

    std::string pbout;
    typesupport_pb_->serialize(buffer.get(), pbout);

    auto slice = grpc::Slice(pbout);
    auto buf = grpc::ByteBuffer(&slice, 1);
    buf.Duplicate();

    std::lock_guard<std::mutex> lock(write_mutex_);
    buffer_queue_.emplace_back(buf);
    if (buffer_queue_.size() == 1) {
      StartWrite(&buf);
    }
  }

  void OnWriteDone(bool) override {
    std::lock_guard<std::mutex> lock(write_mutex_);
    buffer_queue_.pop_front();
    if (!buffer_queue_.empty()) {
      StartWrite(&buffer_queue_.front());
    }
  }

  void OnDone() override { delete this; }

  void OnCancel() override {
    std::cout << "Cancel" << std::endl;
    sub_.reset();
    Finish(grpc::Status::OK);
  }

private:
  std::shared_ptr<DynamicSubscription> sub_;
  const rosidl_typesupport_protobuf::message_type_support_t *typesupport_pb_;
  std::deque<grpc::ByteBuffer> buffer_queue_;
  std::mutex write_mutex_;
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
  RosService(rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node)
      : node_(node) {}

  grpc::ServerGenericBidiReactor *
  CreateReactor(grpc::GenericCallbackServerContext *ctx) override {
    std::cout << ctx->method() << std::endl;
    if (ctx->method() == "/lrf_head_scan.Topic/Subscribe") {
      return new RosSubscribeReactor(node_, "lrf_head_scan");
    }
    return new UnknownReactor;
  }

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_;
};

void register_service() {}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("grpc_bridge");

  google::protobuf::DescriptorPool pool;
  pool.internal_set_underlay(
      google::protobuf::DescriptorPool::generated_pool());
  create_topic_service(pool, "lrf_head_scan", "sensor_msgs/msg/LaserScan");

  grpc::ServerBuilder builder;
  RosService service(node->get_node_topics_interface());
  ProtoServerReflection reflection(pool);
  builder.AddListeningPort("0.0.0.0:8080", grpc::InsecureServerCredentials());
  builder.RegisterService(&reflection);
  builder.RegisterCallbackGenericService(&service);

  auto server = builder.BuildAndStart();

  std::thread spinner([&] { rclcpp::spin(node); });
  server->Wait();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}
