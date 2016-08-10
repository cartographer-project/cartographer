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

cmake_minimum_required (VERSION 2.8.7)

project(cartographer)
set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})

include("${CMAKE_SOURCE_DIR}/cmake/functions.cmake")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  google_add_flag(GOOG_CXX_FLAGS "-O3 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  google_add_flag(GOOG_CXX_FLAGS "-O3 -g -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(FATAL_ERROR "Cartographer is too slow to be useful in debug mode.")
else()
  message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Protobuf REQUIRED)

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("doc")
endif()

set(GMOCK_SRC_DIR "/usr/src/gmock" CACHE STRING "Path to google-mock sources.")

add_subdirectory(${GMOCK_SRC_DIR} "${CMAKE_CURRENT_BINARY_DIR}/gmock")

google_add_flag(GOOG_CXX_FLAGS "-std=c++11")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  google_add_flag(GOOG_CXX_FLAGS "-Weverything")
  google_add_flag(GOOG_CXX_FLAGS "-Werror=deprecated")
  google_add_flag(GOOG_CXX_FLAGS "-Werror=non-pod-varargs")
else()
  google_add_flag(GOOG_CXX_FLAGS "-Wall")
  google_add_flag(GOOG_CXX_FLAGS "-Wpedantic")
endif()

# Turn some warnings into errors. These are defined for clang and gcc.
google_add_flag(GOOG_CXX_FLAGS "-Werror=format-security")
google_add_flag(GOOG_CXX_FLAGS "-Werror=return-type")
google_add_flag(GOOG_CXX_FLAGS "-Werror=uninitialized")

enable_testing()

SET(ALL_LIBRARIES "" CACHE INTERNAL "ALL_LIBRARIES")

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files")
install(DIRECTORY configuration_files DESTINATION share/cartographer/)

add_subdirectory("cartographer")

include(CMakePackageConfigHelpers)

# Create a cartographer-config.cmake file for the use from the install tree
# and install it
set(CARTOGRAPHER_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
list(APPEND CARTOGRAPHER_LIBRARY_DIRS "${LUA_LIBRARY_DIR}")

set(CARTOGRAPHER_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${LUA_INCLUDE_DIR}")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${CERES_INCLUDE_DIRS}")
list(APPEND CARTOGRAPHER_INCLUDE_DIRS "${PROTOBUF_INCLUDE_DIR}")

google_combined_library(cartographer
  SRCS "${ALL_LIBRARIES}"
)

get_property(CARTOGRAPHER_LIBRARY_FILE TARGET cartographer PROPERTY LOCATION)
install(
  FILES
    ${CARTOGRAPHER_LIBRARY_FILE}
  DESTINATION
    lib
)

set(CARTOGRAPHER_LIBRARIES "")
list(APPEND CARTOGRAPHER_LIBRARIES "cartographer")
list(APPEND CARTOGRAPHER_LIBRARIES "${CERES_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${Boost_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${LUA_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "${PROTOBUF_LIBRARIES}")
list(APPEND CARTOGRAPHER_LIBRARIES "webp")

CONFIGURE_PACKAGE_CONFIG_FILE(
  cartographer-config.cmake.in
  "${CMAKE_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake"
  PATH_VARS CARTOGRAPHER_INCLUDE_DIRS CARTOGRAPHER_LIBRARY_DIRS
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

install(
  FILES
    "${CMAKE_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake"
  DESTINATION
    share/cartographer/
)
