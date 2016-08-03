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

#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

namespace {

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

}  // namespace

OptimizationProblem::OptimizationProblem(
    const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
        options)
    : options_(options) {}

OptimizationProblem::~OptimizationProblem() {}

void OptimizationProblem::SetMaxNumIterations(const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void OptimizationProblem::Solve(
    const std::vector<Constraint>& constraints,
    const std::vector<const mapping::Submaps*>& trajectories,
    const std::vector<transform::Rigid2d>& initial_point_cloud_poses,
    std::vector<transform::Rigid2d>* point_cloud_poses,
    std::vector<transform::Rigid2d>* submap_transforms) {
  if (point_cloud_poses->empty()) {
    // Nothing to optimize.
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  std::vector<std::array<double, 3>> C_submaps(submap_transforms->size());
  std::vector<std::array<double, 3>> C_point_clouds(point_cloud_poses->size());
  for (size_t i = 0; i != submap_transforms->size(); ++i) {
    C_submaps[i] = FromPose((*submap_transforms)[i]);
    problem.AddParameterBlock(C_submaps[i].data(), 3);
  }
  for (size_t j = 0; j != point_cloud_poses->size(); ++j) {
    C_point_clouds[j] = FromPose((*point_cloud_poses)[j]);
    problem.AddParameterBlock(C_point_clouds[j].data(), 3);
  }

  // Fix the pose of the first submap.
  problem.SetParameterBlockConstant(C_submaps[0].data());

  // Add cost functions for intra- and inter-submap constraints.
  std::vector<std::pair<Constraint::Tag, ceres::ResidualBlockId>>
      constraints_residual_blocks;
  for (const Constraint& constraint : constraints) {
    CHECK_GE(constraint.i, 0);
    CHECK_LT(constraint.i, submap_transforms->size());
    CHECK_GE(constraint.j, 0);
    CHECK_LT(constraint.j, point_cloud_poses->size());
    constraints_residual_blocks.emplace_back(
        constraint.tag,
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
                new SpaCostFunction(constraint.pose)),
            // Only loop closure constraints should have a loss function.
            constraint.tag == Constraint::INTER_SUBMAP
                ? new ceres::HuberLoss(options_.huber_scale())
                : nullptr,
            C_submaps[constraint.i].data(),
            C_point_clouds[constraint.j].data()));
  }

  // Add penalties for changes between consecutive scans.
  CHECK(!point_cloud_poses->empty());
  const Eigen::DiagonalMatrix<double, 3> consecutive_pose_change_penalty_matrix(
      options_.consecutive_scan_translation_penalty_factor(),
      options_.consecutive_scan_translation_penalty_factor(),
      options_.consecutive_scan_rotation_penalty_factor());
  CHECK_GE(initial_point_cloud_poses.size(), point_cloud_poses->size());
  CHECK_GE(trajectories.size(), point_cloud_poses->size());

  // The poses in initial_point_cloud_poses and point_cloud_poses are
  // interleaved from multiple trajectories (although the points from a given
  // trajectory are in time order). 'last_pose_indices[trajectory]' is the index
  // into 'initial_point_cloud_poses' of the most-recent pose on 'trajectory'.
  std::map<const mapping::Submaps*, int> last_pose_indices;

  for (size_t j = 0; j != point_cloud_poses->size(); ++j) {
    const mapping::Submaps* trajectory = trajectories[j];
    // This pose has a predecessor.
    if (last_pose_indices.count(trajectory) != 0) {
      const int last_pose_index = last_pose_indices[trajectory];
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
              new SpaCostFunction(Constraint::Pose{
                  initial_point_cloud_poses[last_pose_index].inverse() *
                      initial_point_cloud_poses[j],
                  consecutive_pose_change_penalty_matrix})),
          nullptr /* loss function */, C_point_clouds[last_pose_index].data(),
          C_point_clouds[j].data());
    }
    last_pose_indices[trajectory] = j;
  }

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  if (options_.log_residual_histograms()) {
    common::Histogram intra_submap_xy_residuals;
    common::Histogram intra_submap_theta_residuals;
    common::Histogram inter_submap_xy_residuals;
    common::Histogram inter_submap_theta_residuals;
    for (auto constraint_residual_block : constraints_residual_blocks) {
      ceres::Problem::EvaluateOptions options;
      options.apply_loss_function = false;
      options.residual_blocks = {constraint_residual_block.second};
      std::vector<double> residuals;
      problem.Evaluate(options, nullptr, &residuals, nullptr, nullptr);
      CHECK_EQ(3, residuals.size());
      switch (constraint_residual_block.first) {
        case Constraint::INTRA_SUBMAP:
          intra_submap_xy_residuals.Add(common::Pow2(residuals[0]) +
                                        common::Pow2(residuals[1]));
          intra_submap_theta_residuals.Add(common::Pow2(residuals[2]));
          break;
        case Constraint::INTER_SUBMAP:
          inter_submap_xy_residuals.Add(common::Pow2(residuals[0]) +
                                        common::Pow2(residuals[1]));
          inter_submap_theta_residuals.Add(common::Pow2(residuals[2]));
          break;
      }
    }
    LOG(INFO) << "Intra-submap x^2 + y^2 residual histogram:\n"
              << intra_submap_xy_residuals.ToString(10);
    LOG(INFO) << "Intra-submap theta^2 residual histogram:\n"
              << intra_submap_theta_residuals.ToString(10);
    LOG(INFO) << "Inter-submap x^2 + y^2 residual histogram:\n"
              << inter_submap_xy_residuals.ToString(10);
    LOG(INFO) << "Inter-submap theta^2 residual histogram:\n"
              << inter_submap_theta_residuals.ToString(10);
  }
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  for (size_t i = 0; i != submap_transforms->size(); ++i) {
    (*submap_transforms)[i] = ToPose(C_submaps[i]);
  }
  for (size_t j = 0; j != point_cloud_poses->size(); ++j) {
    (*point_cloud_poses)[j] = ToPose(C_point_clouds[j]);
  }
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
