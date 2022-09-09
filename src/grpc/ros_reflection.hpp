#pragma once

#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/config_protobuf.h>

#include "reflection.grpc.pb.h"
#include "reflection.pb.h"

using namespace grpc;

class ProtoServerReflection final
    : public grpc::reflection::v1alpha::ServerReflection::Service {
public:
  ProtoServerReflection()
      : descriptor_pool_(protobuf::DescriptorPool::generated_pool()) {}

  ProtoServerReflection(protobuf::DescriptorPool &descriptor_pool)
      : descriptor_pool_(&descriptor_pool) {}

  // implementation of ServerReflectionInfo(stream ServerReflectionRequest) rpc
  // in ServerReflection service
  grpc::Status
  ServerReflectionInfo(grpc::ServerContext *context,
                       grpc::ServerReaderWriter<
                           grpc::reflection::v1alpha::ServerReflectionResponse,
                           grpc::reflection::v1alpha::ServerReflectionRequest>
                           *stream) override {
    using grpc::reflection::v1alpha::ServerReflectionRequest;
    using grpc::reflection::v1alpha::ServerReflectionResponse;
    ServerReflectionRequest request;
    ServerReflectionResponse response;
    Status status;
    while (stream->Read(&request)) {
      switch (request.message_request_case()) {
      case ServerReflectionRequest::MessageRequestCase::kFileByFilename:
        status = GetFileByName(context, request.file_by_filename(), &response);
        break;
      case ServerReflectionRequest::MessageRequestCase::kFileContainingSymbol:
        status = GetFileContainingSymbol(
            context, request.file_containing_symbol(), &response);
        break;
      case ServerReflectionRequest::MessageRequestCase::
          kFileContainingExtension:
        status = GetFileContainingExtension(
            context, &request.file_containing_extension(), &response);
        break;
      case ServerReflectionRequest::MessageRequestCase::
          kAllExtensionNumbersOfType:
        status = GetAllExtensionNumbers(
            context, request.all_extension_numbers_of_type(),
            response.mutable_all_extension_numbers_response());
        break;
      case ServerReflectionRequest::MessageRequestCase::kListServices:
        status =
            ListService(context, response.mutable_list_services_response());
        break;
      default:
        status = Status(StatusCode::UNIMPLEMENTED, "");
      }

      if (!status.ok()) {
        FillErrorResponse(status, response.mutable_error_response());
      }
      response.set_valid_host(request.host());
      response.set_allocated_original_request(
          new ServerReflectionRequest(request));
      stream->Write(response);
    }

    return Status::OK;
  }

private:
  grpc::Status
  ListService(grpc::ServerContext *context,
              grpc::reflection::v1alpha::ListServiceResponse *response) {
    response->add_service()->set_name(
        "grpc.reflection.v1alpha.ServiceReflection");
    response->add_service()->set_name("lrf_head_scan.Topic");
    return grpc::Status::OK;
  }

  grpc::Status
  GetFileByName(grpc::ServerContext *, const std::string &file_name,
                grpc::reflection::v1alpha::ServerReflectionResponse *response) {
    if (descriptor_pool_ == nullptr) {
      return grpc::Status::CANCELLED;
    }

    const protobuf::FileDescriptor *file_desc =
        descriptor_pool_->FindFileByName(file_name);
    if (file_desc == nullptr) {
      return grpc::Status(grpc::StatusCode::NOT_FOUND, "File not found.");
    }
    std::unordered_set<std::string> seen_files;
    FillFileDescriptorResponse(file_desc, response, &seen_files);
    return grpc::Status::OK;
  }

  grpc::Status GetFileContainingSymbol(
      grpc::ServerContext *, const std::string &symbol,
      grpc::reflection::v1alpha::ServerReflectionResponse *response) {
    if (descriptor_pool_ == nullptr) {
      return grpc::Status::CANCELLED;
    }

    const protobuf::FileDescriptor *file_desc =
        descriptor_pool_->FindFileContainingSymbol(symbol);
    if (file_desc == nullptr) {
      return grpc::Status(grpc::StatusCode::NOT_FOUND, "Symbol not found.");
    }
    std::unordered_set<std::string> seen_files;
    FillFileDescriptorResponse(file_desc, response, &seen_files);
    return grpc::Status::OK;
  }

  grpc::Status GetFileContainingExtension(
      grpc::ServerContext *,
      const grpc::reflection::v1alpha::ExtensionRequest *request,
      grpc::reflection::v1alpha::ServerReflectionResponse *response) {
    if (descriptor_pool_ == nullptr) {
      return grpc::Status::CANCELLED;
    }

    const protobuf::Descriptor *desc =
        descriptor_pool_->FindMessageTypeByName(request->containing_type());
    if (desc == nullptr) {
      return grpc::Status(StatusCode::NOT_FOUND, "Type not found.");
    }

    const protobuf::FieldDescriptor *field_desc =
        descriptor_pool_->FindExtensionByNumber(desc,
                                                request->extension_number());
    if (field_desc == nullptr) {
      return grpc::Status(grpc::StatusCode::NOT_FOUND, "Extension not found.");
    }
    std::unordered_set<std::string> seen_files;
    FillFileDescriptorResponse(field_desc->file(), response, &seen_files);
    return grpc::Status::OK;
  }

  grpc::Status GetAllExtensionNumbers(
      grpc::ServerContext *, const std::string &type,
      grpc::reflection::v1alpha::ExtensionNumberResponse *response) {
    if (descriptor_pool_ == nullptr) {
      return grpc::Status::CANCELLED;
    }

    const protobuf::Descriptor *desc =
        descriptor_pool_->FindMessageTypeByName(type);
    if (desc == nullptr) {
      return grpc::Status(grpc::StatusCode::NOT_FOUND, "Type not found.");
    }

    std::vector<const protobuf::FieldDescriptor *> extensions;
    descriptor_pool_->FindAllExtensions(desc, &extensions);
    for (const auto &value : extensions) {
      response->add_extension_number(value->number());
    }
    response->set_base_type_name(type);
    return grpc::Status::OK;
  }

  void FillFileDescriptorResponse(
      const protobuf::FileDescriptor *file_desc,
      grpc::reflection::v1alpha::ServerReflectionResponse *response,
      std::unordered_set<std::string> *seen_files) {
    if (seen_files->find(file_desc->name()) != seen_files->end()) {
      return;
    }
    seen_files->insert(file_desc->name());

    protobuf::FileDescriptorProto file_desc_proto;
    std::string data;
    file_desc->CopyTo(&file_desc_proto);
    file_desc_proto.SerializeToString(&data);
    response->mutable_file_descriptor_response()->add_file_descriptor_proto(
        data);

    for (int i = 0; i < file_desc->dependency_count(); ++i) {
      FillFileDescriptorResponse(file_desc->dependency(i), response,
                                 seen_files);
    }
  }

  void
  FillErrorResponse(const Status &status,
                    grpc::reflection::v1alpha::ErrorResponse *error_response) {
    error_response->set_error_code(status.error_code());
    error_response->set_error_message(status.error_message());
  }

  const protobuf::DescriptorPool *descriptor_pool_;
  const std::vector<string> *services_;
};