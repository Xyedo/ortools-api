cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(OrtoolsAPI VERSION 1.0 LANGUAGES CXX)
add_subdirectory(routing)

find_package(fmt CONFIG REQUIRED)
find_package(cpr CONFIG REQUIRED)
find_package(Drogon CONFIG REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED HINTS build/vcpkg_installed/x64-linux/lib/protobuf)
find_package(GTest REQUIRED HINTS build/vcpkg_installed/x64-linux/include/gtest)
find_package(gRPC CONFIG REQUIRED)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(USE_HIGHS OFF)
set(USE_COINOR OFF)
set(BUILD_SCIP ON)
set(BUILD_TESTING OFF)
FetchContent_Declare(
  ortools
  GIT_REPOSITORY https://github.com/google/or-tools.git
  GIT_TAG v9.12
)
FetchContent_MakeAvailable(ortools)

include_directories(src)

file(GLOB _LIB_SRC "./src/lib/*.h" "./src/lib/*.cpp")
list(FILTER _LIB_SRC EXCLUDE REGEX "./*_test\\.cpp$")
add_library(OrtoolsLib OBJECT ${_LIB_SRC})
target_link_libraries(OrtoolsLib PRIVATE ortools::ortools)

file(GLOB _DTOS_SRC "./src/dtos/*.h" "./src/dtos/*.cpp")
list(FILTER _DTOS_SRC EXCLUDE REGEX "./*_test\\.cpp$")
add_library(OrtoolsDTO OBJECT ${_DTOS_SRC})
target_link_libraries(OrtoolsDTO PRIVATE routing JsonCpp::JsonCpp OrtoolsLib ortools::ortools)

add_executable(OrtoolsGRPC src/cmd/grpc.cpp)
target_link_libraries(OrtoolsGRPC PRIVATE gRPC::grpc gRPC::grpc++ routing OrtoolsDTO JsonCpp::JsonCpp OrtoolsLib ortools::ortools)

add_executable(OrtoolsREST src/cmd/rest.cpp)
target_link_libraries(OrtoolsREST PRIVATE Drogon::Drogon OrtoolsDTO JsonCpp::JsonCpp OrtoolsLib ortools::ortools)


if(ENABLE_COVERAGE)
  target_link_libraries(OrtoolsLib PRIVATE --coverage)
  target_compile_options(OrtoolsLib PRIVATE --coverage)
  target_link_options(OrtoolsLib PRIVATE --coverage)

  target_link_libraries(OrtoolsDTO PRIVATE --coverage)
  target_compile_options(OrtoolsDTO PRIVATE --coverage)
  target_link_options(OrtoolsDTO PRIVATE --coverage)

  target_link_libraries(OrtoolsGRPC PRIVATE --coverage)
  target_compile_options(OrtoolsGRPC PRIVATE --coverage)
  target_link_options(OrtoolsGRPC PRIVATE --coverage)
endif()

if(ENABLE_TESTING)
  enable_testing()
  file(GLOB _TEST_SRCS "./src/**/*_test.cpp")
  include(GoogleTest)

  foreach(_FULL_FILE_NAME IN LISTS _TEST_SRCS)
    get_filename_component(_NAME ${_FULL_FILE_NAME} NAME_WE)

    if(NOT _NAME)
      message(FATAL_ERROR "no NAME provided")
    endif()

    message(STATUS "Configuring test ${_NAME} ...")
    add_executable(${_NAME} ${_FULL_FILE_NAME})
    target_link_libraries(${_NAME} fmt::fmt routing OrtoolsLib OrtoolsDTO JsonCpp::JsonCpp GTest::gtest_main ortools::ortools)
    gtest_discover_tests(${_NAME})

    gtest_discover_tests(${_NAME}
      DISCOVERY_MODE PRE_TEST
      TEST_PREFIX "memcheck:"
      TEST_WRAPPER "valgrind --tool=memcheck --leak-check=full --error-exitcode=1"
    )

    gtest_discover_tests(${_NAME}
      DISCOVERY_MODE PRE_TEST
      TEST_PREFIX "callgrind:"
      TEST_WRAPPER "valgrind --tool=callgrind --error-exitcode=1"
    )

    gtest_discover_tests(${_NAME}
      DISCOVERY_MODE PRE_TEST
      TEST_PREFIX "helgrind:"
      TEST_WRAPPER "valgrind --tool=helgrind --error-exitcode=1"
    )
  endforeach()
endif()
