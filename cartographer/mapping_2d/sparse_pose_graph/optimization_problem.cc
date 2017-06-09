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

#include <algorithm>
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

void OptimizationProblem::AddImuData(const int trajectory_id,
                                     const common::Time time,
                                     const Eigen::Vector3d& linear_acceleration,
                                     const Eigen::Vector3d& angular_velocity) {
  CHECK_GE(trajectory_id, 0);
  imu_data_.resize(
      std::max(imu_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  imu_data_[trajectory_id].push_back(
      mapping_3d::ImuData{time, linear_acceleration, angular_velocity});
}

void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid2d& initial_point_cloud_pose,
    const transform::Rigid2d& point_cloud_pose) {
  CHECK_GE(trajectory_id, 0);
  node_data_.resize(
      std::max(node_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  node_data_[trajectory_id].push_back(
      NodeData{time, initial_point_cloud_pose, point_cloud_pose});
}

void OptimizationProblem::AddSubmap(const int trajectory_id,
                                    const transform::Rigid2d& submap_pose) {
  CHECK_GE(trajectory_id, 0);
  submap_data_.resize(
      std::max(submap_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  submap_data_[trajectory_id].push_back(SubmapData{submap_pose});
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
  // TODO(hrapp): Move ceres data into SubmapData.
  std::vector<std::deque<std::array<double, 3>>> C_submaps(submap_data_.size());
  std::vector<std::deque<std::array<double, 3>>> C_nodes(node_data_.size());
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      if (trajectory_id == 0 && submap_index == 0) {
        // Fix the pose of the first submap of the first trajectory.
        C_submaps[trajectory_id].push_back(
            FromPose(transform::Rigid2d::Identity()));
        problem.AddParameterBlock(C_submaps[trajectory_id].back().data(), 3);
        problem.SetParameterBlockConstant(
            C_submaps[trajectory_id].back().data());
      } else {
        C_submaps[trajectory_id].push_back(
            FromPose(submap_data_[trajectory_id][submap_index].pose));
        problem.AddParameterBlock(C_submaps[trajectory_id].back().data(), 3);
      }
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    C_nodes[trajectory_id].resize(node_data_[trajectory_id].size());
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      C_nodes[trajectory_id][node_index] =
          FromPose(node_data_[trajectory_id][node_index].point_cloud_pose);
      problem.AddParameterBlock(C_nodes[trajectory_id][node_index].data(), 3);
    }
  }

  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
            new SpaCostFunction(constraint.pose)),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id.trajectory_id)
            .at(constraint.submap_id.submap_index)
            .data(),
        C_nodes.at(constraint.node_id.trajectory_id)
            .at(constraint.node_id.node_index)
            .data());
  }

  // Add penalties for changes between consecutive scans.
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (size_t node_index = 1; node_index < node_data_[trajectory_id].size();
         ++node_index) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
              new SpaCostFunction(Constraint::Pose{
                  transform::Embed3D(node_data_[trajectory_id][node_index - 1]
                                         .initial_point_cloud_pose.inverse() *
                                     node_data_[trajectory_id][node_index]
                                         .initial_point_cloud_pose),
                  options_.consecutive_scan_translation_penalty_factor(),
                  options_.consecutive_scan_rotation_penalty_factor()})),
          nullptr /* loss function */,
          C_nodes[trajectory_id][node_index - 1].data(),
          C_nodes[trajectory_id][node_index].data());
    }
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
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      submap_data_[trajectory_id][submap_index].pose =
          ToPose(C_submaps[trajectory_id][submap_index]);
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      node_data_[trajectory_id][node_index].point_cloud_pose =
          ToPose(C_nodes[trajectory_id][node_index]);
    }
  }
}

const std::vector<std::vector<NodeData>>& OptimizationProblem::node_data()
    const {
  return node_data_;
}

const std::vector<std::vector<SubmapData>>& OptimizationProblem::submap_data()
    const {
  return submap_data_;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
