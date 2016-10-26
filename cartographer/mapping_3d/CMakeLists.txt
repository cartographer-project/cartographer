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

google_library(mapping_3d_acceleration_cost_function
  USES_EIGEN
  HDRS
    acceleration_cost_function.h
)

google_library(mapping_3d_ceres_pose
  USES_CERES
  USES_EIGEN
  SRCS
    ceres_pose.cc
  HDRS
    ceres_pose.h
  DEPENDS
    transform_rigid_transform
)

google_library(mapping_3d_global_trajectory_builder
  SRCS
    global_trajectory_builder.cc
  HDRS
    global_trajectory_builder.h
  DEPENDS
    mapping_3d_local_trajectory_builder
    mapping_3d_proto_local_trajectory_builder_options
    mapping_3d_sparse_pose_graph
    mapping_global_trajectory_builder_interface
)

google_library(mapping_3d_hybrid_grid
  USES_EIGEN
  USES_GLOG
  HDRS
    hybrid_grid.h
  DEPENDS
    common_make_unique
    common_math
    common_port
    mapping_probability_values
)

google_library(mapping_3d_imu_integration
  USES_EIGEN
  USES_GLOG
  SRCS
    imu_integration.cc
  HDRS
    imu_integration.h
  DEPENDS
    common_time
    transform_transform
)

google_library(mapping_3d_kalman_local_trajectory_builder
  USES_GLOG
  SRCS
    kalman_local_trajectory_builder.cc
  HDRS
    kalman_local_trajectory_builder.h
  DEPENDS
    common_make_unique
    common_time
    kalman_filter_pose_tracker
    kalman_filter_proto_pose_tracker_options
    mapping_2d_scan_matching_proto_real_time_correlative_scan_matcher_options
    mapping_3d_local_trajectory_builder_interface
    mapping_3d_motion_filter
    mapping_3d_proto_local_trajectory_builder_options
    mapping_3d_proto_submaps_options
    mapping_3d_scan_matching_ceres_scan_matcher
    mapping_3d_scan_matching_proto_ceres_scan_matcher_options
    mapping_3d_scan_matching_real_time_correlative_scan_matcher
    mapping_3d_submaps
    mapping_global_trajectory_builder_interface
    sensor_laser
    sensor_voxel_filter
)

google_library(mapping_3d_kalman_local_trajectory_builder_options
  SRCS
    kalman_local_trajectory_builder_options.cc
  HDRS
    kalman_local_trajectory_builder_options.h
  DEPENDS
    common_lua_parameter_dictionary
    kalman_filter_pose_tracker
    mapping_2d_scan_matching_real_time_correlative_scan_matcher
    mapping_3d_proto_kalman_local_trajectory_builder_options
)

google_library(mapping_3d_laser_fan_inserter
  USES_EIGEN
  USES_GLOG
  SRCS
    laser_fan_inserter.cc
  HDRS
    laser_fan_inserter.h
  DEPENDS
    mapping_3d_hybrid_grid
    mapping_3d_proto_laser_fan_inserter_options
    mapping_probability_values
    sensor_laser
    sensor_point_cloud
)

google_library(mapping_3d_local_trajectory_builder
  SRCS
    local_trajectory_builder.cc
  HDRS
    local_trajectory_builder.h
  DEPENDS
    common_make_unique
    mapping_3d_kalman_local_trajectory_builder
    mapping_3d_local_trajectory_builder_interface
    mapping_3d_optimizing_local_trajectory_builder
    mapping_3d_proto_local_trajectory_builder_options
)

google_library(mapping_3d_local_trajectory_builder_interface
  HDRS
    local_trajectory_builder_interface.h
  DEPENDS
    common_time
    mapping_3d_submaps
    mapping_global_trajectory_builder_interface
    sensor_laser
    transform_rigid_transform
)

google_library(mapping_3d_local_trajectory_builder_options
  USES_GLOG
  SRCS
    local_trajectory_builder_options.cc
  HDRS
    local_trajectory_builder_options.h
  DEPENDS
    common_lua_parameter_dictionary
    mapping_3d_kalman_local_trajectory_builder_options
    mapping_3d_motion_filter
    mapping_3d_optimizing_local_trajectory_builder_options
    mapping_3d_proto_local_trajectory_builder_options
    mapping_3d_scan_matching_ceres_scan_matcher
    mapping_3d_submaps
    sensor_voxel_filter
)

google_library(mapping_3d_motion_filter
  USES_GLOG
  SRCS
    motion_filter.cc
  HDRS
    motion_filter.h
  DEPENDS
    common_lua_parameter_dictionary
    common_time
    mapping_3d_proto_motion_filter_options
    transform_rigid_transform
    transform_transform
)

google_library(mapping_3d_optimizing_local_trajectory_builder
  USES_GLOG
  SRCS
    optimizing_local_trajectory_builder.cc
  HDRS
    optimizing_local_trajectory_builder.h
  DEPENDS
    common_ceres_solver_options
    common_make_unique
    common_time
    kalman_filter_pose_tracker
    mapping_3d_imu_integration
    mapping_3d_local_trajectory_builder_interface
    mapping_3d_motion_filter
    mapping_3d_proto_local_trajectory_builder_options
    mapping_3d_proto_optimizing_local_trajectory_builder_options
    mapping_3d_rotation_cost_function
    mapping_3d_scan_matching_occupied_space_cost_functor
    mapping_3d_scan_matching_proto_ceres_scan_matcher_options
    mapping_3d_scan_matching_translation_delta_cost_functor
    mapping_3d_submaps
    mapping_3d_translation_cost_function
    mapping_global_trajectory_builder_interface
    sensor_laser
    sensor_voxel_filter
    transform_rigid_transform
    transform_transform
    transform_transform_interpolation_buffer
)

google_library(mapping_3d_optimizing_local_trajectory_builder_options
  SRCS
    optimizing_local_trajectory_builder_options.cc
  HDRS
    optimizing_local_trajectory_builder_options.h
  DEPENDS
    common_lua_parameter_dictionary
    mapping_3d_proto_optimizing_local_trajectory_builder_options
)

google_library(mapping_3d_rotation_cost_function
  USES_EIGEN
  HDRS
    rotation_cost_function.h
)

google_library(mapping_3d_sparse_pose_graph
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
    common_mutex
    common_thread_pool
    common_time
    kalman_filter_pose_tracker
    mapping_3d_sparse_pose_graph_constraint_builder
    mapping_3d_sparse_pose_graph_optimization_problem
    mapping_3d_submaps
    mapping_proto_scan_matching_progress
    mapping_sparse_pose_graph
    mapping_sparse_pose_graph_proto_constraint_builder_options
    mapping_trajectory_connectivity
    sensor_point_cloud
    sensor_voxel_filter
    transform_rigid_transform
    transform_transform
)

google_library(mapping_3d_submaps
  USES_EIGEN
  USES_GLOG
  SRCS
    submaps.cc
  HDRS
    submaps.h
  DEPENDS
    common_math
    common_port
    mapping_2d_laser_fan_inserter
    mapping_2d_probability_grid
    mapping_3d_hybrid_grid
    mapping_3d_laser_fan_inserter
    mapping_3d_proto_submaps_options
    mapping_proto_submap_visualization
    mapping_submaps
    sensor_laser
    transform_transform
)

google_library(mapping_3d_translation_cost_function
  USES_EIGEN
  HDRS
    translation_cost_function.h
)

google_test(mapping_3d_hybrid_grid_test
  SRCS
    hybrid_grid_test.cc
  DEPENDS
    mapping_3d_hybrid_grid
)

google_test(mapping_3d_kalman_local_trajectory_builder_test
  USES_EIGEN
  USES_GLOG
  SRCS
    kalman_local_trajectory_builder_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    common_time
    mapping_3d_hybrid_grid
    mapping_3d_kalman_local_trajectory_builder
    mapping_3d_local_trajectory_builder_options
    sensor_laser
    transform_rigid_transform
    transform_rigid_transform_test_helpers
    transform_transform
)

google_test(mapping_3d_laser_fan_inserter_test
  SRCS
    laser_fan_inserter_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_3d_laser_fan_inserter
)

google_test(mapping_3d_motion_filter_test
  SRCS
    motion_filter_test.cc
  DEPENDS
    common_lua_parameter_dictionary_test_helpers
    mapping_3d_motion_filter
)
