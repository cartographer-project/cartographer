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

#include "cartographer/mapping/2d/pose_graph/optimization_problem_2d.h"

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
#include "cartographer/mapping/2d/pose_graph/landmark_cost_function_2d.h"
#include "cartographer/mapping/2d/pose_graph/spa_cost_function_2d.h"
#include "cartographer/mapping/pose_graph/ceres_pose.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {
namespace {

using ::cartographer::mapping::pose_graph::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using NodeData = OptimizationProblem2D::NodeData;
using SubmapData = OptimizationProblem2D::SubmapData;

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

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeData& prev_node, const NodeData& next_node) {
  const NodeData& closest_node =
      observation.time - prev_node.time < next_node.time - observation.time
          ? prev_node
          : next_node;
  return transform::Embed3D(closest_node.pose) *
         transform::Rigid3d::Rotation(closest_node.gravity_alignment) *
         observation.landmark_to_tracking_transform;
}

void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeData>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem) {
  for (const auto& landmark_node : landmark_nodes) {
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      if (!C_landmarks->count(landmark_id)) {
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data);
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      common::make_unique<ceres::QuaternionParameterization>(),
                      problem));
      }
      problem->AddResidualBlock(
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          nullptr /* loss function */, C_nodes->at(prev->id).data(),
          C_nodes->at(next->id).data(), C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace

OptimizationProblem2D::OptimizationProblem2D(
    const pose_graph::proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem2D::~OptimizationProblem2D() {}

void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  imu_data_.Append(trajectory_id, imu_data);
}

void OptimizationProblem2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void OptimizationProblem2D::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid2d& initial_pose, const transform::Rigid2d& pose,
    const Eigen::Quaterniond& gravity_alignment) {
  node_data_.Append(trajectory_id,
                    NodeData{time, initial_pose, pose, gravity_alignment});
}

void OptimizationProblem2D::InsertTrajectoryNode(
    const NodeId& node_id, const common::Time time,
    const transform::Rigid2d& initial_pose, const transform::Rigid2d& pose,
    const Eigen::Quaterniond& gravity_alignment) {
  node_data_.Insert(node_id,
                    NodeData{time, initial_pose, pose, gravity_alignment});
}

void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
}

void OptimizationProblem2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapData{global_submap_pose});
}

void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapData{global_submap_pose});
}

void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void OptimizationProblem2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::set<int>& frozen_trajectories,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapData.
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.pose));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        SpaCostFunction2D::CreateAutoDiffCostFunction(constraint.pose),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  // Add cost  functions for landmarks.
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem);
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeData& first_node_data = prev_node_it->data;
      prev_node_it = node_it;
      const NodeId second_node_id = node_it->id;
      const NodeData& second_node_data = node_it->data;

      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      const transform::Rigid3d relative_pose =
          ComputeRelativePose(trajectory_id, first_node_data, second_node_data);
      problem.AddResidualBlock(
          SpaCostFunction2D::CreateAutoDiffCostFunction(Constraint::Pose{
              relative_pose, options_.consecutive_node_translation_weight(),
              options_.consecutive_node_rotation_weight()}),
          nullptr /* loss function */, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
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
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).pose = ToPose(C_node_id_data.data);
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

const MapById<NodeId, NodeData>& OptimizationProblem2D::node_data() const {
  return node_data_;
}

const MapById<SubmapId, SubmapData>& OptimizationProblem2D::submap_data()
    const {
  return submap_data_;
}

const std::map<std::string, transform::Rigid3d>&
OptimizationProblem2D::landmark_data() const {
  return landmark_data_;
}

const sensor::MapByTime<sensor::ImuData>& OptimizationProblem2D::imu_data()
    const {
  return imu_data_;
}

const sensor::MapByTime<sensor::OdometryData>&
OptimizationProblem2D::odometry_data() const {
  return odometry_data_;
}

std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return common::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return common::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

transform::Rigid3d OptimizationProblem2D::ComputeRelativePose(
    const int trajectory_id, const NodeData& first_node_data,
    const NodeData& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      return transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
             first_node_odometry->inverse() * (*second_node_odometry) *
             transform::Rigid3d::Rotation(
                 second_node_data.gravity_alignment.inverse());
    }
  }
  return transform::Embed3D(first_node_data.initial_pose.inverse() *
                            second_node_data.initial_pose);
}

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer
