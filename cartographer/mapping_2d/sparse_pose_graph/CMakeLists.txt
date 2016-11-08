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

google_library(mapping_2d_sparse_pose_graph_constraint_builder
  USES_EIGEN
  USES_GLOG
  SRCS
    constraint_builder.cc
  HDRS
    constraint_builder.h
  DEPENDS
    common_fixed_ratio_sampler
    common_histogram
    common_make_unique
    common_math
    common_mutex
    common_thread_pool
    kalman_filter_pose_tracker
    mapping_2d_scan_matching_ceres_scan_matcher
    mapping_2d_scan_matching_fast_correlative_scan_matcher
    mapping_2d_scan_matching_proto_ceres_scan_matcher_options
    mapping_2d_scan_matching_proto_fast_correlative_scan_matcher_options
    mapping_2d_submaps
    mapping_3d_scan_matching_ceres_scan_matcher
    mapping_3d_scan_matching_fast_correlative_scan_matcher
    mapping_sparse_pose_graph
    mapping_sparse_pose_graph_proto_constraint_builder_options
    mapping_trajectory_connectivity
    sensor_point_cloud
    sensor_voxel_filter
    transform_transform
)

google_library(mapping_2d_sparse_pose_graph_optimization_problem
  USES_CERES
  USES_EIGEN
  USES_GLOG
  SRCS
    optimization_problem.cc
  HDRS
    optimization_problem.h
  DEPENDS
    common_ceres_solver_options
    common_histogram
    common_lua_parameter_dictionary
    common_math
    common_port
    mapping_2d_sparse_pose_graph_spa_cost_function
    mapping_2d_submaps
    mapping_sparse_pose_graph
    mapping_sparse_pose_graph_proto_optimization_problem_options
    transform_transform
)

google_library(mapping_2d_sparse_pose_graph_spa_cost_function
  USES_CERES
  USES_EIGEN
  HDRS
    spa_cost_function.h
  DEPENDS
    common_math
    kalman_filter_pose_tracker
    mapping_sparse_pose_graph
    transform_rigid_transform
    transform_transform
)
