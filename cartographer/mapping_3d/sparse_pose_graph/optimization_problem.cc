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

#include "cartographer/mapping_3d/sparse_pose_graph/optimization_problem.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_3d/acceleration_cost_function.h"
#include "cartographer/mapping_3d/ceres_pose.h"
#include "cartographer/mapping_3d/imu_integration.h"
#include "cartographer/mapping_3d/rotation_cost_function.h"
#include "cartographer/mapping_3d/rotation_parameterization.h"
#include "cartographer/mapping_3d/sparse_pose_graph/spa_cost_function.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace sparse_pose_graph {

OptimizationProblem::OptimizationProblem(
    const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
        options,
    FixZ fix_z)
    : options_(options), fix_z_(fix_z) {}

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

void OptimizationProblem::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  CHECK_GE(trajectory_id, 0);
  fixed_frame_pose_data_.resize(std::max(
      fixed_frame_pose_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  fixed_frame_pose_data_[trajectory_id].Push(fixed_frame_pose_data.time,
                                             fixed_frame_pose_data.pose);
}

void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid3d& initial_pose, const transform::Rigid3d& pose) {
  CHECK_GE(trajectory_id, 0);
  node_data_.resize(
      std::max(node_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  node_data_[trajectory_id].push_back(NodeData{time, initial_pose, pose});
}

void OptimizationProblem::AddSubmap(const int trajectory_id,
                                    const transform::Rigid3d& submap_pose) {
  CHECK_GE(trajectory_id, 0);
  submap_data_.resize(
      std::max(submap_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  submap_data_[trajectory_id].push_back(SubmapData{submap_pose});
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

  const auto translation_parameterization =
      [this]() -> std::unique_ptr<ceres::LocalParameterization> {
    return fix_z_ == FixZ::kYes
               ? common::make_unique<ceres::SubsetParameterization>(
                     3, std::vector<int>{2})
               : nullptr;
  };

  // Set the starting point.
  CHECK(!submap_data_.empty());
  CHECK(!submap_data_[0].empty());
  // TODO(hrapp): Move ceres data into SubmapData.
  std::vector<std::deque<CeresPose>> C_submaps(submap_data_.size());
  std::vector<std::deque<CeresPose>> C_nodes(node_data_.size());
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    const bool frozen = frozen_trajectories.count(trajectory_id);
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      if (trajectory_id == 0 && submap_index == 0) {
        // Tie the first submap of the first trajectory to the origin.
        C_submaps[trajectory_id].emplace_back(
            transform::Rigid3d::Identity(), translation_parameterization(),
            common::make_unique<ceres::AutoDiffLocalParameterization<
                ConstantYawQuaternionPlus, 4, 2>>(),
            &problem);
        problem.SetParameterBlockConstant(
            C_submaps[trajectory_id].back().translation());
      } else {
        C_submaps[trajectory_id].emplace_back(
            submap_data_[trajectory_id][submap_index].pose,
            translation_parameterization(),
            common::make_unique<ceres::QuaternionParameterization>(), &problem);
      }
      if (frozen) {
        problem.SetParameterBlockConstant(
            C_submaps[trajectory_id].back().rotation());
        problem.SetParameterBlockConstant(
            C_submaps[trajectory_id].back().translation());
      }
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    const bool frozen = frozen_trajectories.count(trajectory_id);
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      C_nodes[trajectory_id].emplace_back(
          node_data_[trajectory_id][node_index].pose,
          translation_parameterization(),
          common::make_unique<ceres::QuaternionParameterization>(), &problem);
      if (frozen) {
        problem.SetParameterBlockConstant(
            C_nodes[trajectory_id].back().rotation());
        problem.SetParameterBlockConstant(
            C_nodes[trajectory_id].back().translation());
      }
    }
  }

  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SpaCostFunction, 6, 4, 3, 4, 3>(
            new SpaCostFunction(constraint.pose)),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id.trajectory_id)
            .at(constraint.submap_id.submap_index)
            .rotation(),
        C_submaps.at(constraint.submap_id.trajectory_id)
            .at(constraint.submap_id.submap_index)
            .translation(),
        C_nodes.at(constraint.node_id.trajectory_id)
            .at(constraint.node_id.node_index)
            .rotation(),
        C_nodes.at(constraint.node_id.trajectory_id)
            .at(constraint.node_id.node_index)
            .translation());
  }

  // Add constraints based on IMU observations of angular velocities and
  // linear acceleration.
  trajectory_data_.resize(imu_data_.size());
  CHECK_GE(trajectory_data_.size(), node_data_.size());
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    const auto& node_data = node_data_[trajectory_id];
    if (node_data.empty()) {
      // We skip empty trajectories which might not have any IMU data.
      continue;
    }
    TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    problem.AddParameterBlock(trajectory_data.imu_calibration.data(), 4,
                              new ceres::QuaternionParameterization());
    const std::deque<sensor::ImuData>& imu_data = imu_data_.at(trajectory_id);
    CHECK(!imu_data.empty());

    // Skip IMU data before the first node of this trajectory.
    auto it = imu_data.cbegin();
    while ((it + 1) != imu_data.cend() && (it + 1)->time <= node_data[0].time) {
      ++it;
    }

    for (size_t node_index = 1; node_index < node_data.size(); ++node_index) {
      auto it2 = it;
      const IntegrateImuResult<double> result =
          IntegrateImu(imu_data, node_data[node_index - 1].time,
                       node_data[node_index].time, &it);
      if (node_index + 1 < node_data.size()) {
        const common::Time first_time = node_data[node_index - 1].time;
        const common::Time second_time = node_data[node_index].time;
        const common::Time third_time = node_data[node_index + 1].time;
        const common::Duration first_duration = second_time - first_time;
        const common::Duration second_duration = third_time - second_time;
        const common::Time first_center = first_time + first_duration / 2;
        const common::Time second_center = second_time + second_duration / 2;
        const IntegrateImuResult<double> result_to_first_center =
            IntegrateImu(imu_data, first_time, first_center, &it2);
        const IntegrateImuResult<double> result_center_to_center =
            IntegrateImu(imu_data, first_center, second_center, &it2);
        // 'delta_velocity' is the change in velocity from the point in time
        // halfway between the first and second poses to halfway between second
        // and third pose. It is computed from IMU data and still contains a
        // delta due to gravity. The orientation of this vector is in the IMU
        // frame at the second pose.
        const Eigen::Vector3d delta_velocity =
            (result.delta_rotation.inverse() *
             result_to_first_center.delta_rotation) *
            result_center_to_center.delta_velocity;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AccelerationCostFunction, 3, 4, 3,
                                            3, 3, 1, 4>(
                new AccelerationCostFunction(
                    options_.acceleration_weight(), delta_velocity,
                    common::ToSeconds(first_duration),
                    common::ToSeconds(second_duration))),
            nullptr, C_nodes[trajectory_id].at(node_index).rotation(),
            C_nodes[trajectory_id].at(node_index - 1).translation(),
            C_nodes[trajectory_id].at(node_index).translation(),
            C_nodes[trajectory_id].at(node_index + 1).translation(),
            &trajectory_data.gravity_constant,
            trajectory_data.imu_calibration.data());
      }
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<RotationCostFunction, 3, 4, 4, 4>(
              new RotationCostFunction(options_.rotation_weight(),
                                       result.delta_rotation)),
          nullptr, C_nodes[trajectory_id].at(node_index - 1).rotation(),
          C_nodes[trajectory_id].at(node_index).rotation(),
          trajectory_data.imu_calibration.data());
    }
  }

  // Add fixed frame pose constraints.
  std::deque<CeresPose> C_fixed_frames;
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    if (trajectory_id >= fixed_frame_pose_data_.size()) {
      break;
    }

    bool fixed_frame_pose_initialized = false;

    const auto& node_data = node_data_[trajectory_id];
    for (size_t node_index = 0; node_index < node_data.size(); ++node_index) {
      if (!fixed_frame_pose_data_.at(trajectory_id)
               .Has(node_data[node_index].time)) {
        continue;
      }

      const mapping::SparsePoseGraph::Constraint::Pose constraint_pose{
          fixed_frame_pose_data_.at(trajectory_id)
              .Lookup(node_data[node_index].time),
          options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        const transform::Rigid3d fixed_frame_pose_in_map =
            node_data[node_index].pose * constraint_pose.zbar_ij.inverse();
        C_fixed_frames.emplace_back(
            transform::Rigid3d(
                fixed_frame_pose_in_map.translation(),
                Eigen::AngleAxisd(
                    transform::GetYaw(fixed_frame_pose_in_map.rotation()),
                    Eigen::Vector3d::UnitZ())),
            nullptr,
            common::make_unique<ceres::AutoDiffLocalParameterization<
                YawOnlyQuaternionPlus, 4, 1>>(),
            &problem);
        fixed_frame_pose_initialized = true;
      }

      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 6, 4, 3, 4, 3>(
              new SpaCostFunction(constraint_pose)),
          nullptr, C_fixed_frames.back().rotation(),
          C_fixed_frames.back().translation(),
          C_nodes.at(trajectory_id).at(node_index).rotation(),
          C_nodes.at(trajectory_id).at(node_index).translation());
    }
  }

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
    for (size_t trajectory_id = 0; trajectory_id != trajectory_data_.size();
         ++trajectory_id) {
      if (trajectory_id != 0) {
        LOG(INFO) << "Trajectory " << trajectory_id << ":";
      }
      LOG(INFO) << "Gravity was: "
                << trajectory_data_[trajectory_id].gravity_constant;
      const auto& imu_calibration =
          trajectory_data_[trajectory_id].imu_calibration;
      LOG(INFO) << "IMU correction was: "
                << common::RadToDeg(2. * std::acos(imu_calibration[0]))
                << " deg (" << imu_calibration[0] << ", " << imu_calibration[1]
                << ", " << imu_calibration[2] << ", " << imu_calibration[3]
                << ")";
    }
  }

  // Store the result.
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      submap_data_[trajectory_id][submap_index].pose =
          C_submaps[trajectory_id][submap_index].ToRigid();
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      node_data_[trajectory_id][node_index].pose =
          C_nodes[trajectory_id][node_index].ToRigid();
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
}  // namespace mapping_3d
}  // namespace cartographer
