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

google_library(mapping_2d_scan_matching_ceres_scan_matcher
  USES_CERES
  USES_EIGEN
  USES_GLOG
  SRCS
    ceres_scan_matcher.cc
  HDRS
    ceres_scan_matcher.h
  DEPENDS
    common_ceres_solver_options
    common_lua_parameter_dictionary
    kalman_filter_pose_tracker
    mapping_2d_probability_grid
    mapping_2d_scan_matching_occupied_space_cost_functor
    mapping_2d_scan_matching_proto_ceres_scan_matcher_options
    mapping_2d_scan_matching_rotation_delta_cost_functor
    mapping_2d_scan_matching_translation_delta_cost_functor
    sensor_point_cloud
    transform_transform
)

google_library(mapping_2d_scan_matching_correlative_scan_matcher
  USES_EIGEN
  SRCS
    correlative_scan_matcher.cc
  HDRS
    correlative_scan_matcher.h
  DEPENDS
    common_lua_parameter_dictionary
    common_math
    mapping_2d_map_limits
    mapping_2d_xy_index
    sensor_point_cloud
)

google_library(mapping_2d_scan_matching_fast_correlative_scan_matcher
  USES_EIGEN
  USES_GLOG
  SRCS
    fast_correlative_scan_matcher.cc
  HDRS
    fast_correlative_scan_matcher.h
  DEPENDS
    common_math
    common_port
    mapping_2d_probability_grid
    mapping_2d_scan_matching_correlative_scan_matcher
    mapping_2d_scan_matching_proto_fast_correlative_scan_matcher_options
    sensor_point_cloud
    transform_transform
)

google_library(mapping_2d_scan_matching_fast_global_localizer
  USES_EIGEN
  USES_GLOG
  SRCS
    fast_global_localizer.cc
  HDRS
    fast_global_localizer.h
  DEPENDS
    mapping_2d_scan_matching_fast_correlative_scan_matcher
    sensor_voxel_filter
)

google_library(mapping_2d_scan_matching_occupied_space_cost_functor
  USES_CERES
  USES_EIGEN
  HDRS
    occupied_space_cost_functor.h
  DEPENDS
    mapping_2d_probability_grid
    sensor_point_cloud
)

google_library(mapping_2d_scan_matching_real_time_correlative_scan_matcher
  USES_EIGEN
  USES_GLOG
  SRCS
    real_time_correlative_scan_matcher.cc
  HDRS
    real_time_correlative_scan_matcher.h
  DEPENDS
    common_lua_parameter_dictionary
    common_math
    mapping_2d_probability_grid
    mapping_2d_scan_matching_correlative_scan_matcher
    mapping_2d_scan_matching_proto_real_time_correlative_scan_matcher_options
    sensor_point_cloud
    transform_transform
)

google_library(mapping_2d_scan_matching_rotation_delta_cost_functor
  USES_EIGEN
  HDRS
    rotation_delta_cost_functor.h
  DEPENDS
    transform_transform
)

google_library(mapping_2d_scan_matching_translation_delta_cost_functor
  USES_EIGEN
  HDRS
    translation_delta_cost_functor.h
)

google_test(mapping_2d_scan_matching_ceres_scan_matcher_test
  SRCS
    ceres_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    mapping_2d_probability_grid
    mapping_2d_scan_matching_ceres_scan_matcher
    sensor_point_cloud
    transform_rigid_transform_test_helpers
)

google_test(mapping_2d_scan_matching_correlative_scan_matcher_test
  SRCS
    correlative_scan_matcher_test.cc
  DEPENDS
    mapping_2d_scan_matching_correlative_scan_matcher
    sensor_point_cloud
)

google_test(mapping_2d_scan_matching_fast_correlative_scan_matcher_test
  SRCS
    fast_correlative_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_2d_laser_fan_inserter
    mapping_2d_probability_grid
    mapping_2d_scan_matching_fast_correlative_scan_matcher
    transform_rigid_transform_test_helpers
    transform_transform
)

google_test(mapping_2d_scan_matching_real_time_correlative_scan_matcher_test
  USES_EIGEN
  SRCS
    real_time_correlative_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    kalman_filter_pose_tracker
    mapping_2d_laser_fan_inserter
    mapping_2d_probability_grid
    mapping_2d_scan_matching_real_time_correlative_scan_matcher
    sensor_point_cloud
    transform_transform
)
