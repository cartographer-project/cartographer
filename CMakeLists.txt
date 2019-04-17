# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.2)

project(cartographer)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})
option(BUILD_GRPC "build Cartographer gRPC support" false)
set(CARTOGRAPHER_HAS_GRPC ${BUILD_GRPC})
option(BUILD_PROMETHEUS "build Prometheus monitoring support" false)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
if(WIN32)
  list(APPEND BOOST_COMPONENTS zlib)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
if(WIN32)
  # On Windows, Protobuf is incorrectly found by the bundled CMake module, so prefer native CMake config.
  set(protobuf_MODULE_COMPATIBLE TRUE CACHE INTERNAL "")
  find_package(Protobuf 3.0.0 CONFIG)
else()
  find_package(Protobuf 3.0.0 REQUIRED)
endif()

if (${BUILD_GRPC})
  find_package(async_grpc REQUIRED)
endif()

if(${BUILD_PROMETHEUS})
  find_package( ZLIB REQUIRED )
endif()

include(FindPkgConfig)
if (NOT WIN32)
  PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
else()
  find_library(CAIRO_LIBRARIES cairo)
endif()

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("docs")
endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/cartographer/)

install(DIRECTORY cmake DESTINATION share/cartographer/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "cartographer/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "cartographer/*.cc")
file(GLOB_RECURSE TEST_LIBRARY_HDRS "cartographer/fake_*.h" "cartographer/*test_helpers*.h" "cartographer/mock_*.h")
file(GLOB_RECURSE TEST_LIBRARY_SRCS "cartographer/fake_*.cc" "cartographer/*test_helpers*.cc" "cartographer/mock_*.cc")
file(GLOB_RECURSE ALL_TESTS "cartographer/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer/*_main.cc")

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
endif()
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer/cloud/*")
file(GLOB_RECURSE ALL_PROMETHEUS_FILES "cartographer/cloud/metrics/prometheus/*")
list(REMOVE_ITEM ALL_GRPC_FILES ${ALL_PROMETHEUS_FILES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
if (NOT ${BUILD_PROMETHEUS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_PROMETHEUS_FILES})
endif()
set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "cartographer/*/internal/*.h")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

file(GLOB_RECURSE ALL_PROTOS "cartographer/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "cartographer/*_service.proto")
list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_FILES})
endif()

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

if(${BUILD_GRPC})
  set(ALL_GRPC_SERVICE_SRCS)
  set(ALL_GRPC_SERVICE_HDRS)
  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${PROJECT_BINARY_DIR}
        -I ${PROJECT_SOURCE_DIR}
        ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
      VERBATIM
    )
  endforeach()
  set_source_files_properties(${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS} PROPERTIES GENERATED TRUE)
  list(APPEND ALL_LIBRARY_HDRS ${ALL_GRPC_SERVICE_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_GRPC_SERVICE_SRCS})
endif()
set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS} ${ALL_GRPC_SERVICE_HDRS})

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

configure_file(
  ${PROJECT_SOURCE_DIR}/cartographer/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/cartographer/common/config.h)

google_binary(cartographer_autogenerate_ground_truth
  SRCS
    cartographer/ground_truth/autogenerate_ground_truth_main.cc
)

google_binary(cartographer_compute_relations_metrics
  SRCS
    cartographer/ground_truth/compute_relations_metrics_main.cc
)

google_binary(cartographer_pbstream
  SRCS
  cartographer/io/pbstream_main.cc
)

google_binary(cartographer_print_configuration
  SRCS
  cartographer/common/print_configuration_main.cc
)

if(${BUILD_GRPC})
  google_binary(cartographer_grpc_server
    SRCS
      cartographer/cloud/map_builder_server_main.cc
  )
  target_link_libraries(cartographer_grpc_server PUBLIC grpc++)
  target_link_libraries(cartographer_grpc_server PUBLIC async_grpc)
  if(${BUILD_PROMETHEUS})
    target_link_libraries(cartographer_grpc_server PUBLIC ${ZLIB_LIBRARIES})
    target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-core)
    target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-pull)
  endif()
endif()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

if (WIN32)
  find_package(glog REQUIRED)
  set(GLOG_LIBRARY glog::glog)
else()
  set(GLOG_LIBRARY glog)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)
if(WIN32)
  # Needed to fix conflict with MSVC's error macro.
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()
if(MSVC)
  # Needed for VS 2017 5.8
  target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE -D_USE_MATH_DEFINES)
endif()

if("${CAIRO_INCLUDE_DIRS}")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${CAIRO_INCLUDE_DIRS}")
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} 
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)
if (NOT WIN32)
  target_link_libraries(${PROJECT_NAME} PUBLIC pthread)
endif()
if(${BUILD_GRPC})
  target_link_libraries(${PROJECT_NAME} PUBLIC grpc++)
  target_link_libraries(${PROJECT_NAME} PUBLIC async_grpc)
endif()
if(${BUILD_PROMETHEUS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${ZLIB_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} PUBLIC prometheus-cpp-core)
  target_link_libraries(${PROJECT_NAME} PUBLIC prometheus-cpp-pull)
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_PROMETHEUS=1)
endif()

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

set(TEST_LIB
  cartographer_test_library
)
add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
target_include_directories(${TEST_LIB} SYSTEM PRIVATE
  "${GMOCK_INCLUDE_DIRS}")
# Needed for dynamically linked GTest on Windows.
if (WIN32)
  target_compile_definitions(${TEST_LIB} PUBLIC -DGTEST_LINKED_AS_SHARED_LIBRARY)
endif()
target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
set_target_properties(${TEST_LIB} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

foreach(ABS_FIL ${ALL_TESTS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  # Replace slashes as required for CMP0037.
  string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
  google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
  if(${BUILD_GRPC})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++)
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC async_grpc)
  endif()
  if(${BUILD_PROMETHEUS})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${ZLIB_LIBRARIES})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
  endif()
  target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
endforeach()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT CartographerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

set(CARTOGRAPHER_CMAKE_DIR share/cartographer/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cartographer-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  PATH_VARS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

install(
  EXPORT CartographerExport
  DESTINATION share/cartographer/cmake/
  FILE CartographerTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  DESTINATION share/cartographer/
)
