
find_package(Protobuf REQUIRED HINTS ../build/vcpkg_installed/x64-linux/lib/protobuf)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads)

#
# Protobuf/Grpc source files
#
set(PROTO_FILES
    routing-proto/routing.proto
)

#
# Add Library target with protobuf sources
#
add_library(routing ${PROTO_FILES})
target_link_libraries(routing
    PUBLIC
        protobuf::libprotobuf
        gRPC::grpc
        gRPC::grpc++
)
target_include_directories(routing PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

#
# Compile protobuf and grpc files in myproto target to cpp
#
get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET routing LANGUAGE cpp)
protobuf_generate(TARGET routing LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")