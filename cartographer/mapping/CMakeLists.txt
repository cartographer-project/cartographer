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

google_library(mapping_collated_trajectory_builder
  USES_GLOG
  SRCS
    collated_trajectory_builder.cc
  HDRS
    collated_trajectory_builder.h
  DEPENDS
    common_port
    common_rate_timer
    common_time
    mapping_global_trajectory_builder_interface
    mapping_submaps
    mapping_trajectory_builder
    sensor_collator
    sensor_data
)

google_library(mapping_global_trajectory_builder_interface
  HDRS
    global_trajectory_builder_interface.h
  DEPENDS
    common_time
    kalman_filter_pose_tracker
    mapping_submaps
    mapping_trajectory_builder
    sensor_laser
    sensor_point_cloud
)

google_library(mapping_imu_tracker
  USES_EIGEN
  USES_GLOG
  SRCS
    imu_tracker.cc
  HDRS
    imu_tracker.h
  DEPENDS
    common_math
    common_time
    transform_transform
)

google_library(mapping_map_builder
  USES_EIGEN
  USES_GLOG
  SRCS
    map_builder.cc
  HDRS
    map_builder.h
  DEPENDS
    common_lua_parameter_dictionary
    common_make_unique
    common_port
    common_thread_pool
    mapping_2d_global_trajectory_builder
    mapping_2d_sparse_pose_graph
    mapping_3d_global_trajectory_builder
    mapping_3d_local_trajectory_builder_options
    mapping_3d_sparse_pose_graph
    mapping_collated_trajectory_builder
    mapping_proto_map_builder_options
    mapping_sparse_pose_graph
    mapping_submaps
    mapping_trajectory_builder
    mapping_trajectory_node
    sensor_collator
    sensor_laser
    sensor_voxel_filter
    transform_rigid_transform
    transform_transform
)

google_library(mapping_odometry_state_tracker
  SRCS
    odometry_state_tracker.cc
  HDRS
    odometry_state_tracker.h
  DEPENDS
    common_time
    transform_rigid_transform
)

google_library(mapping_probability_values
  USES_GLOG
  SRCS
    probability_values.cc
  HDRS
    probability_values.h
  DEPENDS
    common_math
    common_port
)

google_library(mapping_sparse_pose_graph
  USES_GLOG
  SRCS
    sparse_pose_graph.cc
  HDRS
    sparse_pose_graph.h
  DEPENDS
    common_lua_parameter_dictionary
    kalman_filter_pose_tracker
    mapping_proto_scan_matching_progress
    mapping_proto_sparse_pose_graph
    mapping_proto_sparse_pose_graph_options
    mapping_sparse_pose_graph_constraint_builder
    mapping_sparse_pose_graph_optimization_problem_options
    mapping_submaps
    mapping_trajectory_node
    transform_rigid_transform
    transform_transform
)

google_library(mapping_submaps
  USES_EIGEN
  USES_GLOG
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

google_library(mapping_trajectory_builder
  HDRS
    trajectory_builder.h
  DEPENDS
    common_make_unique
    common_port
    common_time
    kalman_filter_pose_tracker
    mapping_submaps
    sensor_data
    sensor_laser
    sensor_point_cloud
)

google_library(mapping_trajectory_connectivity
  USES_GLOG
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
  SRCS
    trajectory_node.cc
  HDRS
    trajectory_node.h
  DEPENDS
    common_time
    proto_trajectory
    sensor_laser
    transform_rigid_transform
    transform_transform
)

google_test(mapping_probability_values_test
  SRCS
    probability_values_test.cc
  DEPENDS
    mapping_probability_values
)

google_test(mapping_sparse_pose_graph_test
  USES_GLOG
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
