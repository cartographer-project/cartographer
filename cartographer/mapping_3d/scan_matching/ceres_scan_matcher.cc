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

#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"

#include <string>
#include <utility>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping_3d/ceres_pose.h"
#include "cartographer/mapping_3d/rotation_parameterization.h"
#include "cartographer/mapping_3d/scan_matching/occupied_space_cost_functor.h"
#include "cartographer/mapping_3d/scan_matching/rotation_delta_cost_functor.h"
#include "cartographer/mapping_3d/scan_matching/translation_delta_cost_functor.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions options;
  for (int i = 0;; ++i) {
    const std::string lua_identifier =
        "occupied_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_occupied_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_only_optimize_yaw(
      parameter_dictionary->GetBool("only_optimize_yaw"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher::CeresScanMatcher(
    const proto::CeresScanMatcherOptions& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

void CeresScanMatcher::Match(const transform::Rigid3d& previous_pose,
                             const transform::Rigid3d& initial_pose_estimate,
                             const std::vector<PointCloudAndHybridGridPointers>&
                                 point_clouds_and_hybrid_grids,
                             transform::Rigid3d* const pose_estimate,
                             ceres::Solver::Summary* const summary) {
  ceres::Problem problem;
  CeresPose ceres_pose(
      initial_pose_estimate, nullptr /* translation_parameterization */,
      options_.only_optimize_yaw()
          ? std::unique_ptr<ceres::LocalParameterization>(
                common::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>())
          : std::unique_ptr<ceres::LocalParameterization>(
                common::make_unique<ceres::QuaternionParameterization>()),
      &problem);

  CHECK_EQ(options_.occupied_space_weight_size(),
           point_clouds_and_hybrid_grids.size());
  for (size_t i = 0; i != point_clouds_and_hybrid_grids.size(); ++i) {
    CHECK_GT(options_.occupied_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
        *point_clouds_and_hybrid_grids[i].first;
    const HybridGrid& hybrid_grid = *point_clouds_and_hybrid_grids[i].second;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor,
                                        ceres::DYNAMIC, 3, 4>(
            new OccupiedSpaceCostFunctor(
                options_.occupied_space_weight(i) /
                    std::sqrt(static_cast<double>(point_cloud.size())),
                point_cloud, hybrid_grid),
            point_cloud.size()),
        nullptr, ceres_pose.translation(), ceres_pose.rotation());
  }
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 3, 3>(
          new TranslationDeltaCostFunctor(options_.translation_weight(),
                                          previous_pose)),
      nullptr, ceres_pose.translation());
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
          new RotationDeltaCostFunctor(options_.rotation_weight(),
                                       initial_pose_estimate.rotation())),
      nullptr, ceres_pose.rotation());

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = ceres_pose.ToRigid();
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
