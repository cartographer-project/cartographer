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

google_library(mapping_3d_scan_matching_ceres_scan_matcher
  USES_CERES
  USES_EIGEN
  SRCS
    ceres_scan_matcher.cc
  HDRS
    ceres_scan_matcher.h
  DEPENDS
    common_ceres_solver_options
    common_lua_parameter_dictionary
    common_make_unique
    kalman_filter_pose_tracker
    mapping_3d_ceres_pose
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_occupied_space_cost_functor
    mapping_3d_scan_matching_proto_ceres_scan_matcher_options
    mapping_3d_scan_matching_rotation_delta_cost_functor
    mapping_3d_scan_matching_translation_delta_cost_functor
    sensor_point_cloud
    transform_rigid_transform
    transform_transform
)

google_library(mapping_3d_scan_matching_fast_correlative_scan_matcher
  USES_CERES
  USES_EIGEN
  SRCS
    fast_correlative_scan_matcher.cc
  HDRS
    fast_correlative_scan_matcher.h
  DEPENDS
    common_make_unique
    common_math
    common_port
    mapping_2d_scan_matching_fast_correlative_scan_matcher
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_precomputation_grid
    mapping_3d_scan_matching_proto_fast_correlative_scan_matcher_options
    mapping_3d_scan_matching_rotational_scan_matcher
    mapping_trajectory_node
    sensor_point_cloud
    transform_transform
)

google_library(mapping_3d_scan_matching_interpolated_grid
  HDRS
    interpolated_grid.h
  DEPENDS
    mapping_3d_hybrid_grid
)

google_library(mapping_3d_scan_matching_occupied_space_cost_functor
  USES_EIGEN
  HDRS
    occupied_space_cost_functor.h
  DEPENDS
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_interpolated_grid
    sensor_point_cloud
    transform_rigid_transform
    transform_transform
)

google_library(mapping_3d_scan_matching_precomputation_grid
  USES_CERES
  USES_EIGEN
  SRCS
    precomputation_grid.cc
  HDRS
    precomputation_grid.h
  DEPENDS
    common_math
    mapping_3d_hybrid_grid
    mapping_probability_values
)

google_library(mapping_3d_scan_matching_real_time_correlative_scan_matcher
  USES_CERES
  USES_EIGEN
  SRCS
    real_time_correlative_scan_matcher.cc
  HDRS
    real_time_correlative_scan_matcher.h
  DEPENDS
    common_math
    mapping_2d_scan_matching_proto_real_time_correlative_scan_matcher_options
    mapping_3d_hybrid_grid
    sensor_point_cloud
    transform_transform
)

google_library(mapping_3d_scan_matching_rotation_delta_cost_functor
  USES_CERES
  USES_EIGEN
  HDRS
    rotation_delta_cost_functor.h
  DEPENDS
    transform_rigid_transform
    transform_transform
)

google_library(mapping_3d_scan_matching_rotational_scan_matcher
  USES_EIGEN
  SRCS
    rotational_scan_matcher.cc
  HDRS
    rotational_scan_matcher.h
  DEPENDS
    common_math
    mapping_trajectory_node
    sensor_point_cloud
)

google_library(mapping_3d_scan_matching_translation_delta_cost_functor
  USES_EIGEN
  HDRS
    translation_delta_cost_functor.h
  DEPENDS
    transform_rigid_transform
)

google_test(mapping_3d_scan_matching_ceres_scan_matcher_test
  USES_EIGEN
  SRCS
    ceres_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_ceres_scan_matcher
    sensor_point_cloud
    transform_rigid_transform
    transform_rigid_transform_test_helpers
)

google_test(mapping_3d_scan_matching_fast_correlative_scan_matcher_test
  SRCS
    fast_correlative_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_3d_laser_fan_inserter
    mapping_3d_scan_matching_fast_correlative_scan_matcher
    transform_rigid_transform_test_helpers
    transform_transform
)

google_test(mapping_3d_scan_matching_interpolated_grid_test
  USES_EIGEN
  SRCS
    interpolated_grid_test.cc
  DEPENDS
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_interpolated_grid
)

google_test(mapping_3d_scan_matching_precomputation_grid_test
  SRCS
    precomputation_grid_test.cc
  DEPENDS
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_precomputation_grid
)

google_test(mapping_3d_scan_matching_real_time_correlative_scan_matcher_test
  USES_EIGEN
  SRCS
    real_time_correlative_scan_matcher_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_2d_scan_matching_real_time_correlative_scan_matcher
    mapping_3d_hybrid_grid
    mapping_3d_scan_matching_real_time_correlative_scan_matcher
    sensor_point_cloud
    transform_rigid_transform
    transform_rigid_transform_test_helpers
    transform_transform
)
