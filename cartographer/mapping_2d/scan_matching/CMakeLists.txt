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

google_test(mapping_2d_scan_matching_ceres_scan_matcher_test
  SRCS
    ceres_scan_matcher_test.cc
)

google_test(mapping_2d_scan_matching_correlative_scan_matcher_test
  SRCS
    correlative_scan_matcher_test.cc
)

google_test(mapping_2d_scan_matching_fast_correlative_scan_matcher_test
  SRCS
    fast_correlative_scan_matcher_test.cc
)

google_test(mapping_2d_scan_matching_real_time_correlative_scan_matcher_test
  SRCS
    real_time_correlative_scan_matcher_test.cc
)
