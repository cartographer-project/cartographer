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

#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/occupied_space_cost_functor.h"
#include "cartographer/mapping_2d/scan_matching/rotation_delta_cost_functor.h"
#include "cartographer/mapping_2d/scan_matching/translation_delta_cost_functor.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_covariance_scale(
      parameter_dictionary->GetDouble("covariance_scale"));
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

CeresScanMatcher::~CeresScanMatcher() {}

void CeresScanMatcher::Match(const transform::Rigid2d& previous_pose,
                             const transform::Rigid2d& initial_pose_estimate,
                             const sensor::PointCloud& point_cloud,
                             const ProbabilityGrid& probability_grid,
                             transform::Rigid2d* const pose_estimate,
                             kalman_filter::Pose2DCovariance* const covariance,
                             ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor, ceres::DYNAMIC,
                                      3>(
          new OccupiedSpaceCostFunctor(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, probability_grid),
          point_cloud.size()),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 2, 3>(
          new TranslationDeltaCostFunctor(options_.translation_weight(),
                                          previous_pose)),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 1, 3>(
          new RotationDeltaCostFunctor(options_.rotation_weight(),
                                       ceres_pose_estimate[2])),
      nullptr, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);

  ceres::Covariance::Options options;
  ceres::Covariance covariance_computer(options);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(ceres_pose_estimate, ceres_pose_estimate);
  CHECK(covariance_computer.Compute(covariance_blocks, &problem));
  double ceres_covariance[3 * 3];
  covariance_computer.GetCovarianceBlock(ceres_pose_estimate,
                                         ceres_pose_estimate, ceres_covariance);
  *covariance = Eigen::Map<kalman_filter::Pose2DCovariance>(ceres_covariance);
  *covariance *= options_.covariance_scale();
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
