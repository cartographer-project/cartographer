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

add_subdirectory("proto")

google_library(transform_rigid_transform
  USES_EIGEN
  USES_GLOG
  SRCS
    rigid_transform.cc
  HDRS
    rigid_transform.h
  DEPENDS
    common_lua_parameter_dictionary
    common_math
    common_port
)

google_library(transform_rigid_transform_test_helpers
  USES_EIGEN
  HDRS
    rigid_transform_test_helpers.h
  DEPENDS
    common_port
    transform_rigid_transform
)

google_library(transform_transform
  USES_EIGEN
  SRCS
    transform.cc
  HDRS
    transform.h
  DEPENDS
    common_math
    transform_proto_transform
    transform_rigid_transform
)

google_library(transform_transform_interpolation_buffer
  USES_EIGEN
  USES_GLOG
  SRCS
    transform_interpolation_buffer.cc
  HDRS
    transform_interpolation_buffer.h
  DEPENDS
    common_make_unique
    common_time
    proto_trajectory
    transform_rigid_transform
    transform_transform
)

google_test(transform_transform_interpolation_buffer_test
  USES_EIGEN
  SRCS
    transform_interpolation_buffer_test.cc
  DEPENDS
    transform_rigid_transform
    transform_rigid_transform_test_helpers
    transform_transform_interpolation_buffer
)

google_test(transform_transform_test
  SRCS
    transform_test.cc
  DEPENDS
    transform_rigid_transform
    transform_rigid_transform_test_helpers
    transform_transform
)
