#pragma once

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <grpcpp/grpcpp.h>
#include <map>
#include <rclcpp/rclcpp.hpp>

#include <rcpputils/shared_library.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rosidl_typesupport_protobuf/message_type_support.hpp>
#include <string>
#include <tuple>
#include <utility>

// From rclcpp
std::tuple<std::string, std::string, std::string>
extract_type_identifier(const std::string &full_type) {
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos || sep_position_back == 0 ||
      sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
        "Message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module = full_type.substr(
        sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

std::map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> libs;

void create_topic_service(google::protobuf::DescriptorPool &pool,
                          std::string service, std::string type) {
  auto proto_identifier = "rosidl_typesupport_protobuf_cpp";
  auto proto_ts_lib = rclcpp::get_typesupport_library(type, proto_identifier);
  libs.emplace(type, proto_ts_lib);
  auto proto_ts_handle =
      rclcpp::get_typesupport_handle(type, proto_identifier, *proto_ts_lib);
  auto typesupport_pb_ =
      static_cast<const rosidl_typesupport_protobuf::message_type_support_t *>(
          proto_ts_handle->data);

  sensor_msgs::msg::LaserScan msg;
  std::string pbout;
  typesupport_pb_->serialize(&msg, pbout);

  // Create Subscribe descriptor
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) =
      extract_type_identifier(type);

  auto pb_type = "." + package_name + "." + middle_module + ".pb." + type_name;

  auto generated_pool = google::protobuf::DescriptorPool::generated_pool();
  generated_pool->FindFileByName(type + ".proto");

  google::protobuf::FileDescriptorProto proto_desc;
  proto_desc.set_name(service + "/Topic.proto");
  proto_desc.set_package(service);
  proto_desc.set_syntax("proto3");
  proto_desc.add_dependency(type + ".proto");

  auto request_type = proto_desc.add_message_type();
  request_type->set_name("Request");

  auto service_desc = proto_desc.add_service();
  service_desc->set_name("Topic");

  auto subscribe_desc = service_desc->add_method();
  subscribe_desc->set_name("Subscribe");
  subscribe_desc->set_input_type("." + service + ".Request");
  subscribe_desc->set_output_type(pb_type);
  subscribe_desc->set_server_streaming(true);

  pool.BuildFile(proto_desc);
}