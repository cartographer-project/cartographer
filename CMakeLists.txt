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

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(cartographer)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})

include("${CMAKE_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Protobuf REQUIRED)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("docs")
endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files")
install(DIRECTORY configuration_files DESTINATION share/cartographer/)

install(DIRECTORY cmake DESTINATION share/cartographer/)

include(CMakePackageConfigHelpers)

# Create a cartographer-config.cmake file for the use from the install tree
# and install it
set(CARTOGRAPHER_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
list(APPEND CARTOGRAPHER_LIBRARY_DIRS "${LUA_LIBRARY_DIR}")

set(CARTOGRAPHER_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${LUA_INCLUDE_DIR}")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${CERES_INCLUDE_DIRS}")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${PROTOBUF_INCLUDE_DIR}")

set(CARTOGRAPHER_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake")

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${CMAKE_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${CMAKE_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${CMAKE_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${CMAKE_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${CMAKE_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${CMAKE_BINARY_DIR} -I
      ${CMAKE_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

add_library(cartographer ${ALL_SRCS})
add_subdirectory("cartographer")

target_include_directories(cartographer SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(cartographer PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(cartographer SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(cartographer PUBLIC ${CERES_LIBRARIES})

target_include_directories(cartographer SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_link_libraries(cartographer PUBLIC ${LUA_LIBRARIES})

target_include_directories(cartographer SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(cartographer PUBLIC ${Boost_LIBRARIES})

target_link_libraries(cartographer PUBLIC webp)

target_link_libraries(cartographer PUBLIC glog)

target_link_libraries(cartographer PUBLIC gflags)

target_include_directories(cartographer SYSTEM PUBLIC
  "${CAIRO_INCLUDE_DIRS}")
target_link_libraries(cartographer PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(cartographer SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(cartographer PUBLIC ${PROTOBUF_LIBRARY} pthread)

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(cartographer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# TODO(damonkohler): Create a testing library.
target_include_directories(cartographer SYSTEM PRIVATE
  "${GMOCK_INCLUDE_DIRS}")
target_link_libraries(cartographer PUBLIC ${GMOCK_LIBRARIES})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(cartographer PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS cartographer
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install source headers.
file(GLOB_RECURSE hdrs "*.h")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${CMAKE_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${CMAKE_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${CMAKE_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${CMAKE_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

set(CARTOGRAPHER_LIBRARIES "")
list(APPEND CARTOGRAPHER_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/libcartographer.a")
list(APPEND CARTOGRAPHER_LIBRARIES "${CERES_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${Boost_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${LUA_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${PROTOBUF_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${CAIRO_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "webp")

CONFIGURE_PACKAGE_CONFIG_FILE(
  cartographer-config.cmake.in
  "${CMAKE_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake"
  PATH_VARS CARTOGRAPHER_INCLUDE_DIRS CARTOGRAPHER_LIBRARY_DIRS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

install(
  FILES
    "${CMAKE_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake"
  DESTINATION
    share/cartographer/
)
