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

google_library(mapping_sparse_pose_graph_constraint_builder
  SRCS
    constraint_builder.cc
  HDRS
    constraint_builder.h
  DEPENDS
    common_lua_parameter_dictionary
    mapping_2d_scan_matching_ceres_scan_matcher
    mapping_2d_scan_matching_fast_correlative_scan_matcher
    mapping_3d_scan_matching_ceres_scan_matcher
    mapping_3d_scan_matching_fast_correlative_scan_matcher
    mapping_sparse_pose_graph_proto_constraint_builder_options
    sensor_voxel_filter
)

google_library(mapping_sparse_pose_graph_optimization_problem_options
  SRCS
    optimization_problem_options.cc
  HDRS
    optimization_problem_options.h
  DEPENDS
    common_ceres_solver_options
    common_lua_parameter_dictionary
    mapping_sparse_pose_graph_proto_optimization_problem_options
)
