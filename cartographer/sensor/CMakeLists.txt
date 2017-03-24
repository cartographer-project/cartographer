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

google_test(sensor_collator_test
  SRCS
    collator_test.cc
)

google_test(sensor_compressed_point_cloud_test
  SRCS
    compressed_point_cloud_test.cc
)

google_test(sensor_range_data_test
  SRCS
    range_data_test.cc
)

google_test(sensor_ordered_multi_queue_test
  SRCS
    ordered_multi_queue_test.cc
)

google_test(sensor_point_cloud_test
  SRCS
    point_cloud_test.cc
)

google_test(sensor_voxel_filter_test
  SRCS
    voxel_filter_test.cc
)
