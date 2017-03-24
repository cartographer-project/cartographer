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

google_test(mapping_2d_range_data_inserter_test
  SRCS
    range_data_inserter_test.cc
)

google_test(mapping_2d_map_limits_test
  SRCS
    map_limits_test.cc
)

google_test(mapping_2d_probability_grid_test
  SRCS
    probability_grid_test.cc
)

google_test(mapping_2d_sparse_pose_graph_test
  SRCS
    sparse_pose_graph_test.cc
)

google_test(mapping_2d_submaps_test
  SRCS
    submaps_test.cc
)

google_test(mapping_2d_xy_index_test
  SRCS
    xy_index_test.cc
)
