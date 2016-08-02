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

google_proto_library(mapping_2d_scan_matching_proto_ceres_scan_matcher_options
  SRCS
    ceres_scan_matcher_options.proto
  DEPENDS
    common_proto_ceres_solver_options
)

google_proto_library(mapping_2d_scan_matching_proto_fast_correlative_scan_matcher_options
  SRCS
    fast_correlative_scan_matcher_options.proto
)

google_proto_library(mapping_2d_scan_matching_proto_real_time_correlative_scan_matcher_options
  SRCS
    real_time_correlative_scan_matcher_options.proto
)
