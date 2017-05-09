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
#include "cartographer/mapping_2d/sparse_pose_graph/spa_cost_function.h"
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

void OptimizationProblem::AddImuData(const mapping::Submaps* const trajectory,
                                     const common::Time time,
                                     const Eigen::Vector3d& linear_acceleration,
                                     const Eigen::Vector3d& angular_velocity) {
  imu_data_[trajectory].push_back(
      mapping_3d::ImuData{time, linear_acceleration, angular_velocity});
}

void OptimizationProblem::AddTrajectoryNode(
    const mapping::Submaps* const trajectory, const common::Time time,
    const transform::Rigid2d& initial_point_cloud_pose,
    const transform::Rigid2d& point_cloud_pose) {
  node_data_.push_back(
      NodeData{trajectory, time, initial_point_cloud_pose, point_cloud_pose});
}

void OptimizationProblem::AddSubmap(const mapping::Submaps* const trajectory,
                                    const transform::Rigid2d& submap_pose) {
  submap_data_.push_back(SubmapData{trajectory, submap_pose});
}

void OptimizationProblem::SetMaxNumIterations(const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void OptimizationProblem::Solve(const std::vector<Constraint>& constraints) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  std::vector<std::array<double, 3>> C_submaps(submap_data_.size());
  std::vector<std::array<double, 3>> C_point_clouds(node_data_.size());
  for (size_t i = 0; i != submap_data_.size(); ++i) {
    C_submaps[i] = FromPose(submap_data_[i].pose);
    problem.AddParameterBlock(C_submaps[i].data(), 3);
  }
  for (size_t j = 0; j != node_data_.size(); ++j) {
    C_point_clouds[j] = FromPose(node_data_[j].point_cloud_pose);
    problem.AddParameterBlock(C_point_clouds[j].data(), 3);
  }

  // Fix the pose of the first submap.
  problem.SetParameterBlockConstant(C_submaps[0].data());

  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    CHECK_GE(constraint.i, 0);
    CHECK_LT(constraint.i, submap_data_.size());
    CHECK_GE(constraint.j, 0);
    CHECK_LT(constraint.j, node_data_.size());
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
            new SpaCostFunction(constraint.pose)),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps[constraint.i].data(), C_point_clouds[constraint.j].data());
  }

  // Add penalties for changes between consecutive scans.
  const Eigen::DiagonalMatrix<double, 3> consecutive_pose_change_penalty_matrix(
      options_.consecutive_scan_translation_penalty_factor(),
      options_.consecutive_scan_translation_penalty_factor(),
      options_.consecutive_scan_rotation_penalty_factor());

  // The poses in 'node_data_' are interleaved from multiple trajectories
  // (although the points from a given trajectory are in time order).
  // 'last_pose_indices[trajectory]' is the index of the most-recent pose on
  // 'trajectory'.
  std::map<const mapping::Submaps*, int> last_pose_indices;

  for (size_t j = 0; j != node_data_.size(); ++j) {
    const mapping::Submaps* trajectory = node_data_[j].trajectory;
    // This pose has a predecessor.
    if (last_pose_indices.count(trajectory) != 0) {
      const int last_pose_index = last_pose_indices[trajectory];
      constexpr double kUnusedPositionPenalty = 1.;
      constexpr double kUnusedOrientationPenalty = 1.;
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
              new SpaCostFunction(Constraint::Pose{
                  transform::Embed3D(node_data_[last_pose_index]
                                         .initial_point_cloud_pose.inverse() *
                                     node_data_[j].initial_point_cloud_pose),
                  kalman_filter::Embed3D(consecutive_pose_change_penalty_matrix,
                                         kUnusedPositionPenalty,
                                         kUnusedOrientationPenalty)})),
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

  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  for (size_t i = 0; i != submap_data_.size(); ++i) {
    submap_data_[i].pose = ToPose(C_submaps[i]);
  }
  for (size_t j = 0; j != node_data_.size(); ++j) {
    node_data_[j].point_cloud_pose = ToPose(C_point_clouds[j]);
  }
}

const std::vector<NodeData>& OptimizationProblem::node_data() const {
  return node_data_;
}

const std::vector<SubmapData>& OptimizationProblem::submap_data() const {
  return submap_data_;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
