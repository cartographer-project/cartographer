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

google_proto_library(mapping_2d_proto_laser_fan_inserter_options
  SRCS
    laser_fan_inserter_options.proto
)

google_proto_library(mapping_2d_proto_local_trajectory_builder_options
  SRCS
    local_trajectory_builder_options.proto
  DEPENDS
    mapping_2d_proto_submaps_options
    mapping_2d_scan_matching_proto_ceres_scan_matcher_options
    mapping_2d_scan_matching_proto_real_time_correlative_scan_matcher_options
    mapping_3d_proto_motion_filter_options
    sensor_proto_adaptive_voxel_filter_options
)

google_proto_library(mapping_2d_proto_submaps_options
  SRCS
    submaps_options.proto
  DEPENDS
    mapping_2d_proto_laser_fan_inserter_options
)
