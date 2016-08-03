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
add_subdirectory("sparse_pose_graph")

google_library(mapping_global_trajectory_builder_interface
  SRCS
    global_trajectory_builder_interface.cc
  HDRS
    global_trajectory_builder_interface.h
  DEPENDS
    common_time
    kalman_filter_pose_tracker
    mapping_submaps
    sensor_laser
    sensor_point_cloud
)

google_library(mapping_probability_values
  USES_CERES
  SRCS
    probability_values.cc
  HDRS
    probability_values.h
  DEPENDS
    common_math
    common_port
)

google_library(mapping_sensor_collator
  USES_CERES
  USES_EIGEN
  HDRS
    sensor_collator.h
  DEPENDS
    common_make_unique
    common_ordered_multi_queue
    common_time
    sensor_sensor_packet_period_histogram_builder
)

google_library(mapping_sparse_pose_graph
  USES_CERES
  SRCS
    sparse_pose_graph.cc
  HDRS
    sparse_pose_graph.h
  DEPENDS
    common_lua_parameter_dictionary
    mapping_proto_scan_matching_progress
    mapping_proto_sparse_pose_graph_options
    mapping_sparse_pose_graph_constraint_builder
    mapping_sparse_pose_graph_optimization_problem_options
    mapping_submaps
    mapping_trajectory_node
    transform_rigid_transform
)

google_library(mapping_submaps
  USES_CERES
  USES_EIGEN
  SRCS
    submaps.cc
  HDRS
    submaps.h
  DEPENDS
    common_math
    common_port
    mapping_2d_probability_grid
    mapping_probability_values
    mapping_proto_submaps
    mapping_trajectory_node
    transform_transform
)

google_library(mapping_trajectory_connectivity
  USES_CERES
  SRCS
    trajectory_connectivity.cc
  HDRS
    trajectory_connectivity.h
  DEPENDS
    common_mutex
    mapping_proto_trajectory_connectivity
    mapping_submaps
)

google_library(mapping_trajectory_node
  USES_EIGEN
  HDRS
    trajectory_node.h
  DEPENDS
    common_time
    sensor_laser
    transform_rigid_transform
)

google_test(mapping_probability_values_test
  SRCS
    probability_values_test.cc
  DEPENDS
    mapping_probability_values
)

google_test(mapping_sensor_collator_test
  SRCS
    sensor_collator_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    common_time
    mapping_sensor_collator
    sensor_proto_sensor
)

google_test(mapping_sparse_pose_graph_test
  USES_CERES
  SRCS
    sparse_pose_graph_test.cc
  DEPENDS
    mapping_sparse_pose_graph
)

google_test(mapping_submaps_test
  SRCS
    submaps_test.cc
  DEPENDS
    mapping_submaps
)

google_test(mapping_trajectory_connectivity_test
  SRCS
    trajectory_connectivity_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_2d_submaps
    mapping_trajectory_connectivity
)
