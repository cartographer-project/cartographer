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
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Protobuf REQUIRED)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("docs")
endif()

SET(ALL_LIBRARIES "" CACHE INTERNAL "ALL_LIBRARIES")

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files")
install(DIRECTORY configuration_files DESTINATION share/cartographer/)

install(DIRECTORY cmake DESTINATION share/cartographer/)

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

set(CARTOGRAPHER_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake")

google_combined_library(cartographer
  SRCS "${ALL_LIBRARIES}"
)

get_property(CARTOGRAPHER_LIBRARY_FILE TARGET cartographer PROPERTY LOCATION)
get_filename_component(CARTOGRAPHER_LIBRARY_FILE_BASENAME 
  ${CARTOGRAPHER_LIBRARY_FILE} NAME)
install(
  FILES
    ${CARTOGRAPHER_LIBRARY_FILE}
  DESTINATION
    lib
)

set(CARTOGRAPHER_LIBRARIES "")
list(APPEND CARTOGRAPHER_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/${CARTOGRAPHER_LIBRARY_FILE_BASENAME}")
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
