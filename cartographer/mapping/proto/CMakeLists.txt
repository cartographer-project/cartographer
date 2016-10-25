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

google_proto_library(mapping_proto_map_builder_options
  SRCS
    map_builder_options.proto
  DEPENDS
    mapping_2d_proto_local_trajectory_builder_options
    mapping_3d_proto_local_trajectory_builder_options
    mapping_proto_sparse_pose_graph_options
)

google_proto_library(mapping_proto_scan_matching_progress
  SRCS
    scan_matching_progress.proto
)

google_proto_library(mapping_proto_sparse_pose_graph
  SRCS
    sparse_pose_graph.proto
  DEPENDS
    proto_trajectory
    transform_proto_transform
)

google_proto_library(mapping_proto_sparse_pose_graph_options
  SRCS
    sparse_pose_graph_options.proto
  DEPENDS
    mapping_sparse_pose_graph_proto_constraint_builder_options
    mapping_sparse_pose_graph_proto_optimization_problem_options
)

google_proto_library(mapping_proto_submaps
  SRCS
    submaps.proto
  DEPENDS
    transform_proto_transform
)

google_proto_library(mapping_proto_trajectory_connectivity
  SRCS
    trajectory_connectivity.proto
)
