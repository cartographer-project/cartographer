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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.h"

#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions3D CreateLocalTrajectoryBuilderOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions3D options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_num_accumulated_range_data(
      parameter_dictionary->GetInt("num_accumulated_range_data"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  *options.mutable_high_resolution_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("high_resolution_adaptive_voxel_filter")
              .get());
  *options.mutable_low_resolution_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("low_resolution_adaptive_voxel_filter")
              .get());
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_real_time_correlative_scan_matcher_options() =
      mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      mapping::scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() = CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  options.set_rotational_histogram_size(
      parameter_dictionary->GetInt("rotational_histogram_size"));
  *options.mutable_submaps_options() = CreateSubmapsOptions3D(
      parameter_dictionary->GetDictionary("submaps").get());
  return options;
}

}  // namespace mapping
}  // namespace cartographer
