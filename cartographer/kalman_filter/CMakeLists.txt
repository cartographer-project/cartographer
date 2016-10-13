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

google_library(kalman_filter_gaussian_distribution
  USES_EIGEN
  HDRS
    gaussian_distribution.h
)

google_library(kalman_filter_pose_tracker
  USES_EIGEN
  USES_GLOG
  SRCS
    pose_tracker.cc
  HDRS
    pose_tracker.h
  DEPENDS
    common_lua_parameter_dictionary
    common_math
    common_port
    common_time
    kalman_filter_gaussian_distribution
    kalman_filter_proto_pose_tracker_options
    kalman_filter_unscented_kalman_filter
    mapping_imu_tracker
    mapping_odometry_state_tracker
    sensor_proto_sensor
    transform_transform
)

google_library(kalman_filter_unscented_kalman_filter
  USES_EIGEN
  USES_GLOG
  HDRS
    unscented_kalman_filter.h
  DEPENDS
    kalman_filter_gaussian_distribution
)

google_test(kalman_filter_gaussian_distribution_test
  SRCS
    gaussian_distribution_test.cc
  DEPENDS
    kalman_filter_gaussian_distribution
)

google_test(kalman_filter_pose_tracker_test
  SRCS
    pose_tracker_test.cc
  DEPENDS
    common_lua_parameter_dictionary
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
    kalman_filter_pose_tracker
    transform_rigid_transform
    transform_rigid_transform_test_helpers
)

google_test(kalman_filter_unscented_kalman_filter_test
  SRCS
    unscented_kalman_filter_test.cc
  DEPENDS
    kalman_filter_gaussian_distribution
    kalman_filter_unscented_kalman_filter
)
