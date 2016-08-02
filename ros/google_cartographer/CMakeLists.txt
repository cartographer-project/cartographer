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

cmake_minimum_required(VERSION 2.8)

project(google_cartographer)

set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  google_cartographer_msgs
  roscpp
  rviz
  sensor_msgs
  tf2
  tf2_eigen
)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    ${PACKAGE_DEPENDENCIES}
)

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(cartographer REQUIRED)
include_directories(${CARTOGRAPHER_INCLUDE_DIRS})
link_directories(${CARTOGRAPHER_LIBRARY_DIRS})

find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})

# TODO(pedrofernandez): CMake test rules are written differently in the core
# library. Change the way it's done here and reenable testing.
# enable_testing()
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(Boost REQUIRED
 COMPONENTS
 system
 iostreams
)
add_definitions(${BOOST_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(cartographer_node
  src/cartographer_node_main.cc
  src/node_constants.h
  src/msg_conversion.h
  src/msg_conversion.cc
  src/time_conversion.h
  src/time_conversion.cc
)
target_link_libraries(cartographer_node
  ${CARTOGRAPHER_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(cartographer_rviz_submaps_visualization
  src/drawable_submap.cc
  src/drawable_submap.h
  src/submaps_display.cc
  src/submaps_display.h
  src/trajectory.h
  src/trajectory.cc
)
target_link_libraries(cartographer_rviz_submaps_visualization
  ${Boost_LIBRARIES}
  ${CARTOGRAPHER_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${ZLIB_LIBRARIES}
)

add_executable(time_conversion_test
  src/time_conversion_test.cc
  src/time_conversion.h
  src/time_conversion.cc
)

target_link_libraries(time_conversion_test
  ${GTEST_BOTH_LIBRARIES}
  ${CARTOGRAPHER_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_test(time_conversion_test time_conversion_test)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/
)

install(DIRECTORY configuration_files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration_files/
)

install(TARGETS
  cartographer_rviz_submaps_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ogre_media}
)
