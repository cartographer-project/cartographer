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

SPARSE_POSE_GRAPH = {
  optimize_every_n_scans = 90,
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    adaptive_voxel_filter = {
      max_length = 0.9,
      min_num_points = 100,
      max_range = 50.,
    },
    min_score = 0.55,
    global_localization_min_score = 0.6,
    lower_covariance_eigenvalue_bound = 1e-11,
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      covariance_scale = 1e-4,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      rotational_histogram_size = 120,
      min_rotational_score = 0.77,
      linear_xy_search_window = 4.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      covariance_scale = 1e-6,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e4,
    rotation_weight = 3e6,
    consecutive_scan_translation_penalty_factor = 1e5,
    consecutive_scan_rotation_penalty_factor = 1e5,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
}
