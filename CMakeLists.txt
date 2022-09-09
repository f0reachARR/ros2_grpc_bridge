cmake_minimum_required(VERSION 3.8)
project(ros2_grpc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_protobuf REQUIRED)

# gRPC related
include(common.cmake)

# Generate reflection proto
get_filename_component(reflection_proto "src/reflection.proto" ABSOLUTE)
get_filename_component(reflection_proto_path "${reflection_proto}" PATH)

set(reflection_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/reflection.pb.cc")
set(reflection_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/reflection.pb.h")
set(reflection_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/reflection.grpc.pb.cc")
set(reflection_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/reflection.grpc.pb.h")
add_custom_command(
      OUTPUT "${reflection_proto_srcs}" "${reflection_proto_hdrs}" "${reflection_grpc_srcs}" "${reflection_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${reflection_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${reflection_proto}"
      DEPENDS "${reflection_proto}")

# Node
add_executable(ros2_grpc_bridge src/ros2_grpc_bridge.cpp ${reflection_proto_srcs} ${reflection_grpc_srcs})
target_include_directories(ros2_grpc_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR})
target_compile_features(ros2_grpc_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ros2_grpc_bridge
  "rclcpp"
  "rosidl_typesupport_introspection_cpp"
  "rosidl_typesupport_protobuf"
)
target_link_libraries(
  ros2_grpc_bridge
  gRPC::grpc++
  ${PROTOBUF_LIBRARIES}
  ${Protobuf_PROTOC_LIBRARIES}
)

install(TARGETS ros2_grpc_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
