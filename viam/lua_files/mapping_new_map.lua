-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "trajectory_builder.lua"
include "map_builder.lua"

-- ++++ ALWAYS TRY TO TUNE FOR A PLATFORM, NOT A PARTICULAR BAG ++++ --

-- ===== Local SLAM Options ======
-- no reason to change these:
TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
TRAJECTORY_BUILDER.trajectory_builder_2d.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER.trajectory_builder_2d.min_range = 0.2
TRAJECTORY_BUILDER.trajectory_builder_2d.max_range = 25.
TRAJECTORY_BUILDER.trajectory_builder_2d.missing_data_ray_length = 25 -- DO NOT CHANGE

-- tuneable:
TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 100

-- ===== Global SLAM Options ======
-- no reason to change these:
MAP_BUILDER.use_trajectory_builder_2d = true
-- tuneable:
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 3

-- ===== Return Options ======
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

return options


-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
-- POSE_GRAPH.optimize_every_n_nodes = 35

-- HAD THESE at some point:

-- TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.40 -- 0.49
-- TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65 -- 0.55
-- TRAJECTORY_BUILDER.trajectory_builder_2d.motion_filter.max_distance_meters = 0
-- TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.occupied_space_weight = 5.
-- TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.translation_weight = 10
-- TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.rotation_weight = 10

-- MAP_BUILDER.pose_graph.constraint_builder.log_match = true
-- MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 5
-- MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.9
-- MAP_BUILDER.pose_graph.constraint_builder.loop_closure_translation_weight = 1
-- MAP_BUILDER.pose_graph.constraint_builder.loop_closure_rotation_weight = 4
-- MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 20
-- MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.75


-- MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.occupied_space_weight = 0.1
-- MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight = 0.1
-- MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight = 0.1
-- MAP_BUILDER.pose_graph.matcher_translation_weight = 0.1
-- MAP_BUILDER.pose_graph.matcher_rotation_weight = 0.1
-- MAP_BUILDER.pose_graph.optimization_problem.odometry_translation_weight = 1
-- MAP_BUILDER.pose_graph.optimization_problem.odometry_rotation_weight = 1
-- MAP_BUILDER.pose_graph.matcher_rotation_weight = 1
-- MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 100
-- MAP_BUILDER.pose_graph.optimization_problem.local_slam_pose_translation_weight = 1e8
-- MAP_BUILDER.pose_graph.optimization_problem.local_slam_pose_rotation_weight = 1e8
-- MAP_BUILDER.pose_graph.optimization_problem.huber_scale = 1e4