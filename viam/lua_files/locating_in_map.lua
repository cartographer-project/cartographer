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
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

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