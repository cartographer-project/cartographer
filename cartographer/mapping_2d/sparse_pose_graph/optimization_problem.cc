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
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
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
                                     const sensor::ImuData& imu_data) {
  CHECK_GE(trajectory_id, 0);
  imu_data_.resize(
      std::max(imu_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  imu_data_[trajectory_id].push_back(imu_data);
}

void OptimizationProblem::AddOdometerData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  CHECK_GE(trajectory_id, 0);
  odometry_data_.resize(
      std::max(odometry_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  odometry_data_[trajectory_id].Push(odometry_data.time, odometry_data.pose);
}

void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid2d& initial_pose, const transform::Rigid2d& pose,
    const Eigen::Quaterniond& gravity_alignment) {
  CHECK_GE(trajectory_id, 0);
  node_data_.resize(
      std::max(node_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  trajectory_data_.resize(std::max(trajectory_data_.size(), node_data_.size()));
  auto& trajectory_data = trajectory_data_[trajectory_id];
  node_data_[trajectory_id].emplace(
      trajectory_data.next_node_index,
      NodeData{time, initial_pose, pose, gravity_alignment});
  ++trajectory_data.next_node_index;
}

void OptimizationProblem::TrimTrajectoryNode(const mapping::NodeId& node_id) {
  auto& node_data = node_data_.at(node_id.trajectory_id);
  CHECK(node_data.erase(node_id.node_index));

  if (!node_data.empty() &&
      node_id.trajectory_id < static_cast<int>(imu_data_.size())) {
    const common::Time node_time = node_data.begin()->second.time;
    auto& imu_data = imu_data_.at(node_id.trajectory_id);
    while (imu_data.size() > 1 && imu_data[1].time <= node_time) {
      imu_data.pop_front();
    }
  }
}

void OptimizationProblem::AddSubmap(const int trajectory_id,
                                    const transform::Rigid2d& submap_pose) {
  submap_data_.Append(trajectory_id, SubmapData{submap_pose});
}

void OptimizationProblem::TrimSubmap(const mapping::SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void OptimizationProblem::SetMaxNumIterations(const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void OptimizationProblem::Solve(const std::vector<Constraint>& constraints,
                                const std::set<int>& frozen_trajectories) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapData.
  mapping::MapById<mapping::SubmapId, std::array<double, 3>> C_submaps;
  std::vector<std::map<int, std::array<double, 3>>> C_nodes(node_data_.size());
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id, FromPose(submap_id_data.data.pose));
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    const bool frozen = frozen_trajectories.count(trajectory_id) != 0;
    for (const auto& index_node_data : node_data_[trajectory_id]) {
      const int node_index = index_node_data.first;
      const NodeData& node_data = index_node_data.second;
      C_nodes[trajectory_id].emplace(node_index, FromPose(node_data.pose));
      problem.AddParameterBlock(C_nodes[trajectory_id].at(node_index).data(),
                                3);
      if (frozen) {
        problem.SetParameterBlockConstant(
            C_nodes[trajectory_id].at(node_index).data());
      }
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
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id.trajectory_id)
            .at(constraint.node_id.node_index)
            .data());
  }

  // Add penalties for violating odometry or changes between consecutive scans
  // if odometry is not available.
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    if (node_data_[trajectory_id].empty()) {
      continue;
    }

    for (auto node_it = node_data_[trajectory_id].begin();;) {
      const int node_index = node_it->first;
      const NodeData& node_data = node_it->second;
      ++node_it;
      if (node_it == node_data_[trajectory_id].end()) {
        break;
      }

      const int next_node_index = node_it->first;
      const NodeData& next_node_data = node_it->second;

      if (next_node_index != node_index + 1) {
        continue;
      }

      const bool odometry_available =
          trajectory_id < odometry_data_.size() &&
          odometry_data_[trajectory_id].Has(next_node_data.time) &&
          odometry_data_[trajectory_id].Has(node_data.time);
      const transform::Rigid3d relative_pose =
          odometry_available
              ? transform::Rigid3d::Rotation(node_data.gravity_alignment) *
                    odometry_data_[trajectory_id]
                        .Lookup(node_data.time)
                        .inverse() *
                    odometry_data_[trajectory_id].Lookup(next_node_data.time) *
                    transform::Rigid3d::Rotation(
                        next_node_data.gravity_alignment.inverse())
              : transform::Embed3D(node_data.initial_pose.inverse() *
                                   next_node_data.initial_pose);
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3, 3, 3>(
              new SpaCostFunction(Constraint::Pose{
                  relative_pose,
                  options_.consecutive_scan_translation_penalty_factor(),
                  options_.consecutive_scan_rotation_penalty_factor()})),
          nullptr /* loss function */,
          C_nodes[trajectory_id].at(node_index).data(),
          C_nodes[trajectory_id].at(next_node_index).data());
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
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).pose = ToPose(C_submap_id_data.data);
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (auto& index_node_data : node_data_[trajectory_id]) {
      index_node_data.second.pose =
          ToPose(C_nodes[trajectory_id].at(index_node_data.first));
    }
  }
}

const std::vector<std::map<int, NodeData>>& OptimizationProblem::node_data()
    const {
  return node_data_;
}

const mapping::MapById<mapping::SubmapId, SubmapData>&
OptimizationProblem::submap_data() const {
  return submap_data_;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
