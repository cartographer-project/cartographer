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


google_binary(cartographer_compute_relations_metrics
  USES_GFLAGS
  USES_GLOG
  SRCS
    compute_relations_metrics_main.cc
  DEPENDS
    common_math
    common_port
    mapping_proto_trajectory
    transform_rigid_transform
    transform_transform
    transform_transform_interpolation_buffer
)
