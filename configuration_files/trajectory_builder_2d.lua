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

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  laser_min_range = 0.,
  laser_max_range = 30.,
  laser_min_z = -0.8,
  laser_max_z = 2.,
  laser_missing_echo_ray_length = 5.,
  laser_voxel_filter_size = 0.025,

  use_online_correlative_scan_matching = false,
  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1e1,
    translation_weight = 1e1,
    rotation_weight = 1e2,
    covariance_scale = 1e-2,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,
  num_odometry_states = 1000,

  submaps = {
    resolution = 0.05,
    half_length = 200.,
    num_laser_fans = 90,
    output_debug_images = false,
    laser_fan_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },
}
