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
add_subdirectory("scan_matching")
add_subdirectory("sparse_pose_graph")

google_library(mapping_2d_global_trajectory_builder
  SRCS
    global_trajectory_builder.cc
  HDRS
    global_trajectory_builder.h
  DEPENDS
    mapping_2d_local_trajectory_builder
    mapping_2d_sparse_pose_graph
    mapping_global_trajectory_builder_interface
)

google_library(mapping_2d_laser_fan_inserter
  USES_EIGEN
  USES_GLOG
  SRCS
    laser_fan_inserter.cc
  HDRS
    laser_fan_inserter.h
  DEPENDS
    common_lua_parameter_dictionary
    common_port
    mapping_2d_probability_grid
    mapping_2d_proto_laser_fan_inserter_options
    mapping_2d_ray_casting
    mapping_2d_xy_index
    sensor_laser
    sensor_point_cloud
)

google_library(mapping_2d_local_trajectory_builder
  SRCS
    local_trajectory_builder.cc
  HDRS
    local_trajectory_builder.h
  DEPENDS
    common_lua_parameter_dictionary
    common_make_unique
    common_time
    kalman_filter_pose_tracker
    mapping_2d_proto_local_trajectory_builder_options
    mapping_2d_scan_matching_ceres_scan_matcher
    mapping_2d_scan_matching_real_time_correlative_scan_matcher
    mapping_2d_submaps
    mapping_3d_motion_filter
    mapping_global_trajectory_builder_interface
    sensor_configuration
    sensor_laser
    sensor_voxel_filter
    transform_rigid_transform
)

google_library(mapping_2d_map_limits
  USES_EIGEN
  USES_GLOG
  HDRS
    map_limits.h
  DEPENDS
    common_math
    mapping_2d_xy_index
    mapping_trajectory_node
    sensor_laser
    sensor_point_cloud
    transform_rigid_transform
    transform_transform
)

google_library(mapping_2d_probability_grid
  USES_GLOG
  HDRS
    probability_grid.h
  DEPENDS
    common_math
    common_port
    mapping_2d_map_limits
    mapping_2d_xy_index
    mapping_probability_values
)

google_library(mapping_2d_ray_casting
  SRCS
    ray_casting.cc
  HDRS
    ray_casting.h
  DEPENDS
    mapping_2d_map_limits
    mapping_2d_xy_index
    sensor_laser
    sensor_point_cloud
    transform_transform
)

google_library(mapping_2d_sparse_pose_graph
  USES_EIGEN
  USES_GLOG
  SRCS
    sparse_pose_graph.cc
  HDRS
    sparse_pose_graph.h
  DEPENDS
    common_fixed_ratio_sampler
    common_make_unique
    common_math
    common_thread_pool
    common_time
    kalman_filter_pose_tracker
    mapping_2d_sparse_pose_graph_constraint_builder
    mapping_2d_sparse_pose_graph_optimization_problem
    mapping_2d_submaps
    mapping_proto_scan_matching_progress
    mapping_sparse_pose_graph
    mapping_sparse_pose_graph_proto_constraint_builder_options
    mapping_trajectory_connectivity
    sensor_compressed_point_cloud
    sensor_point_cloud
    sensor_voxel_filter
    transform_transform
)

google_library(mapping_2d_submaps
  USES_EIGEN
  USES_GLOG
  USES_WEBP
  SRCS
    submaps.cc
  HDRS
    submaps.h
  DEPENDS
    common_lua_parameter_dictionary
    common_make_unique
    common_port
    mapping_2d_laser_fan_inserter
    mapping_2d_map_limits
    mapping_2d_probability_grid
    mapping_2d_proto_submaps_options
    mapping_proto_submaps
    mapping_submaps
    mapping_trajectory_node
    sensor_laser
    transform_rigid_transform
)

google_library(mapping_2d_xy_index
  USES_EIGEN
  USES_GLOG
  HDRS
    xy_index.h
  DEPENDS
    common_math
    common_port
)

google_test(mapping_2d_laser_fan_inserter_test
  SRCS
    laser_fan_inserter_test.cc
  DEPENDS
    common_lua_parameter_dictionary
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    mapping_2d_laser_fan_inserter
    mapping_2d_probability_grid
)

google_test(mapping_2d_map_limits_test
  SRCS
    map_limits_test.cc
  DEPENDS
    mapping_2d_map_limits
)

google_test(mapping_2d_probability_grid_test
  SRCS
    probability_grid_test.cc
  DEPENDS
    mapping_2d_probability_grid
)

google_test(mapping_2d_sparse_pose_graph_test
  SRCS
    sparse_pose_graph_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    common_thread_pool
    common_time
    mapping_2d_laser_fan_inserter
    mapping_2d_sparse_pose_graph
    mapping_2d_submaps
    transform_rigid_transform
    transform_rigid_transform_test_helpers
    transform_transform
)

google_test(mapping_2d_submaps_test
  SRCS
    submaps_test.cc
  DEPENDS
    common_lua_parameter_dictionary
    common_lua_parameter_dictionary_test_helpers
    common_port
    mapping_2d_submaps
    transform_transform
)

google_test(mapping_2d_xy_index_test
  SRCS
    xy_index_test.cc
  DEPENDS
    mapping_2d_xy_index
)
