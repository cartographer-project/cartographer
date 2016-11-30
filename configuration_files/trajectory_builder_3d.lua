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

MAX_3D_LASER_RANGE = 60.

TRAJECTORY_BUILDER_3D = {
  laser_min_range = 1.,
  laser_max_range = MAX_3D_LASER_RANGE,
  scans_per_accumulation = 1,
  laser_voxel_filter_size = 0.15,

  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_LASER_RANGE,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 5.,
    occupied_space_weight_1 = 30.,
    translation_weight = 0.3,
    rotation_weight = 2e3,
    covariance_scale = 2.34e-4,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.004,
  },

  submaps = {
    high_resolution = 0.10,
    high_resolution_max_range = 20.,
    low_resolution = 0.45,
    num_laser_fans = 160,
    laser_fan_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      num_free_space_voxels = 2,
    },
  },

  use = "KALMAN",  -- or "OPTIMIZING".
  kalman_local_trajectory_builder = {
    pose_tracker = {
      orientation_model_variance = 5e-3,
      position_model_variance = 0.00654766,
      velocity_model_variance = 0.53926,
      -- This disables gravity alignment in local SLAM.
      imu_gravity_time_constant = 1e9,
      imu_gravity_variance = 0,
      num_odometry_states = 1,
    },

    use_online_correlative_scan_matching = false,
    real_time_correlative_scan_matcher = {
      linear_search_window = 0.15,
      angular_search_window = math.rad(1.),
      translation_delta_cost_weight = 1e-1,
      rotation_delta_cost_weight = 1e-1,
    },

    odometer_translational_variance = 1e-7,
    odometer_rotational_variance = 1e-7,
  },

  optimizing_local_trajectory_builder = {
    high_resolution_grid_weight = 5.,
    low_resolution_grid_weight = 15.,
    velocity_weight = 4e1,
    translation_weight = 1e2,
    rotation_weight = 1e3,
    odometry_translation_weight = 1e4,
    odometry_rotation_weight = 1e4,
  },
}
