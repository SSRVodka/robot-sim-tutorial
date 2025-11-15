cmake_minimum_required(VERSION 3.12)
project(servoc-demo)

set(PROJ_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${PROJ_ROOT}/grpc-ohdist")

set(GRPC_HOSTDIST ${PROJ_ROOT}/grpc-dist/bin)
set(GRPC_LIBDIST ${PROJ_ROOT}/grpc-ohdist)
set(Protobuf_DIR ${GRPC_LIBDIST}/lib/cmake/protobuf)
set(gRPC_DIR ${GRPC_LIBDIST}/lib/cmake/grpc)
set(absl_DIR ${GRPC_LIBDIST}/lib/cmake/absl)
set(re2_DIR ${GRPC_LIBDIST}/lib/cmake/re2)
set(utf8_range_DIR ${GRPC_LIBDIST}/lib/cmake/utf8_range)

message(STATUS "cmake prefix path: ${CMAKE_PREFIX_PATH}")

find_package(Threads REQUIRED)
option(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

find_program(_PROTOBUF_PROTOC protoc
    PATHS ${GRPC_HOSTDIST}
    NO_DEFAULT_PATH
)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin
    PATHS ${GRPC_HOSTDIST}
    NO_DEFAULT_PATH
)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)

set(ENTRY_LIBNAME "entry")
set(PROJ_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DOHOS_PLATFORM)

# Proto file
get_filename_component(servo_proto "../protos/servo_control.proto" ABSOLUTE)
get_filename_component(servo_proto_path "${servo_proto}" PATH)
get_filename_component(agent_proto "../protos/agent_control.proto" ABSOLUTE)
get_filename_component(agent_proto_path "${agent_proto}" PATH)

function(generate_grpc_sources proto_name proto_path output_prefix output_dir)
    get_filename_component(proto_basename ${proto_name} NAME_WE)
    set(${output_prefix}_proto_srcs "${output_dir}/${proto_basename}.pb.cc")
    set(${output_prefix}_proto_hdrs "${output_dir}/${proto_basename}.pb.h")
    set(${output_prefix}_grpc_srcs "${output_dir}/${proto_basename}.grpc.pb.cc")
    set(${output_prefix}_grpc_hdrs "${output_dir}/${proto_basename}.grpc.pb.h")

    add_custom_command(
        OUTPUT 
            "${${output_prefix}_proto_srcs}" 
            "${${output_prefix}_proto_hdrs}" 
            "${${output_prefix}_grpc_srcs}" 
            "${${output_prefix}_grpc_hdrs}"
        COMMAND ${_PROTOBUF_PROTOC}
        ARGS --grpc_out "${output_dir}"
            --cpp_out "${output_dir}"
            -I "${proto_path}"
            --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
            "${proto_name}"
        DEPENDS "${proto_name}"
    )

    # grpc_proto
    add_library(${output_prefix}_grpc_proto
        ${${output_prefix}_grpc_srcs}
        ${${output_prefix}_grpc_hdrs}
        ${${output_prefix}_proto_srcs}
        ${${output_prefix}_proto_hdrs})
    target_link_libraries(${output_prefix}_grpc_proto
        absl::check
        ${_REFLECTION}
        ${_GRPC_GRPCPP}
        ${_PROTOBUF_LIBPROTOBUF})

    set(${output_prefix}_grpc_proto "${${output_prefix}_grpc_proto}" PARENT_SCOPE)
endfunction()

# Generated sources
set(gen_proto_dir "${CMAKE_CURRENT_BINARY_DIR}")
# Include generated *.pb.h files
include_directories(${gen_proto_dir})
generate_grpc_sources(${servo_proto} ${servo_proto_path} servo ${gen_proto_dir})
generate_grpc_sources(${agent_proto} ${agent_proto_path} agent ${gen_proto_dir})


set(ARCH "x86_64")

include_directories(
    ${PROJ_ROOT_PATH}
    ${PROJ_ROOT_PATH}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(${ENTRY_LIBNAME} SHARED
    napi_init.cpp
)


target_link_directories(${ENTRY_LIBNAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/${ARCH}/lib)

target_link_libraries(${ENTRY_LIBNAME} PUBLIC
    servo_grpc_proto
    agent_grpc_proto
    absl::check
    absl::flags
    absl::flags_parse
    absl::log
    absl::log_initialize
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})

# NDK dylib, oh logger & oh file utilities
target_link_libraries(${ENTRY_LIBNAME} PUBLIC libace_napi.z.so ace_ndk.z hilog_ndk.z libohfileuri.so c++_shared)

target_link_options(${ENTRY_LIBNAME} PRIVATE "-Wl,-rpath,$ORIGIN:$ORIGIN/lib")