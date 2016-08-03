/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping_3d/kalman_local_trajectory_builder_options.h"

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"

namespace cartographer {
namespace mapping_3d {

proto::KalmanLocalTrajectoryBuilderOptions
CreateKalmanLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::KalmanLocalTrajectoryBuilderOptions options;
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping_2d::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_pose_tracker_options() =
      kalman_filter::CreatePoseTrackerOptions(
          parameter_dictionary->GetDictionary("pose_tracker").get());
  return options;
}

}  // namespace mapping_3d
}  // namespace cartographer
