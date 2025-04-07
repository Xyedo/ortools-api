cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(OrtoolsAPI VERSION 1.0 LANGUAGES CXX)
add_subdirectory(routing)

find_package(fmt CONFIG REQUIRED)
find_package(cpr CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED HINTS build/vcpkg_installed/x64-linux/lib/protobuf)
find_package(GTest REQUIRED HINTS build/vcpkg_installed/x64-linux/include/gtest)

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
add_library(OrtoolsAPI_lib OBJECT ${_LIB_SRC})
target_link_libraries(OrtoolsAPI_lib PRIVATE fmt::fmt ortools::ortools)

add_executable(OrtoolsAPI src/main.cpp)
target_link_libraries(OrtoolsAPI PRIVATE fmt::fmt cpr::cpr nlohmann_json routing OrtoolsAPI_lib)


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
  target_link_libraries(${_NAME} fmt::fmt OrtoolsAPI_lib GTest::gtest_main)
  gtest_discover_tests(${_NAME})
endforeach()