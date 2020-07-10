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

include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}
