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

add_subdirectory("proto")

google_library(sensor_compressed_point_cloud
  USES_EIGEN
  SRCS
    compressed_point_cloud.cc
  HDRS
    compressed_point_cloud.h
  DEPENDS
    common_math
    common_port
    mapping_3d_hybrid_grid
    sensor_point_cloud
    sensor_proto_sensor
)

google_library(sensor_configuration
  USES_EIGEN
  USES_GLOG
  SRCS
    configuration.cc
  HDRS
    configuration.h
  DEPENDS
    common_lua_parameter_dictionary
    sensor_proto_configuration
    transform_proto_transform
    transform_rigid_transform
    transform_transform
)

google_library(sensor_laser
  SRCS
    laser.cc
  HDRS
    laser.h
  DEPENDS
    common_port
    sensor_compressed_point_cloud
    sensor_point_cloud
    sensor_proto_sensor
    transform_transform
)

google_library(sensor_point_cloud
  USES_EIGEN
  USES_GLOG
  SRCS
    point_cloud.cc
  HDRS
    point_cloud.h
  DEPENDS
    sensor_proto_sensor
    transform_rigid_transform
    transform_transform
)

google_library(sensor_sensor_packet_period_histogram_builder
  USES_GLOG
  SRCS
    sensor_packet_period_histogram_builder.cc
  HDRS
    sensor_packet_period_histogram_builder.h
  DEPENDS
    common_histogram
    common_port
)

google_library(sensor_voxel_filter
  SRCS
    voxel_filter.cc
  HDRS
    voxel_filter.h
  DEPENDS
    common_lua_parameter_dictionary
    common_math
    mapping_3d_hybrid_grid
    sensor_point_cloud
    sensor_proto_adaptive_voxel_filter_options
)

google_test(sensor_compressed_point_cloud_test
  SRCS
    compressed_point_cloud_test.cc
  DEPENDS
    sensor_compressed_point_cloud
)

google_test(sensor_laser_test
  SRCS
    laser_test.cc
  DEPENDS
    sensor_laser
)

google_test(sensor_point_cloud_test
  SRCS
    point_cloud_test.cc
  DEPENDS
    sensor_point_cloud
)

google_test(sensor_voxel_filter_test
  SRCS
    voxel_filter_test.cc
  DEPENDS
    sensor_voxel_filter
)
