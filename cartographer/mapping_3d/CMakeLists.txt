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

add_subdirectory("scan_matching")
add_subdirectory("sparse_pose_graph")

google_test(mapping_3d_hybrid_grid_test
  SRCS
    hybrid_grid_test.cc
)

google_test(mapping_3d_kalman_local_trajectory_builder_test
  SRCS
    kalman_local_trajectory_builder_test.cc
)

google_test(mapping_3d_laser_fan_inserter_test
  SRCS
    laser_fan_inserter_test.cc
)

google_test(mapping_3d_motion_filter_test
  SRCS
    motion_filter_test.cc
)
