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

#include "cartographer/mapping/internal/constraints/constraint_builder.h"

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace mapping {
namespace constraints {

proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::ConstraintBuilderOptions options;
  options.set_sampling_ratio(parameter_dictionary->GetDouble("sampling_ratio"));
  options.set_max_constraint_distance(
      parameter_dictionary->GetDouble("max_constraint_distance"));
  options.set_min_score(parameter_dictionary->GetDouble("min_score"));
  options.set_global_localization_min_score(
      parameter_dictionary->GetDouble("global_localization_min_score"));
  options.set_loop_closure_translation_weight(
      parameter_dictionary->GetDouble("loop_closure_translation_weight"));
  options.set_loop_closure_rotation_weight(
      parameter_dictionary->GetDouble("loop_closure_rotation_weight"));
  options.set_log_matches(parameter_dictionary->GetBool("log_matches"));
  *options.mutable_fast_correlative_scan_matcher_options() =
      scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("fast_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_fast_correlative_scan_matcher_options_3d() =
      scan_matching::CreateFastCorrelativeScanMatcherOptions3D(
          parameter_dictionary
              ->GetDictionary("fast_correlative_scan_matcher_3d")
              .get());
  *options.mutable_ceres_scan_matcher_options_3d() =
      scan_matching::CreateCeresScanMatcherOptions3D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher_3d").get());
  return options;
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
