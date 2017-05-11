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
#include "cartographer/mapping_3d/rotation_cost_function.h"
#include "cartographer/mapping_3d/sparse_pose_graph/spa_cost_function.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace sparse_pose_graph {

namespace {

struct ConstantYawQuaternionPlus {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    const T delta_norm =
        ceres::sqrt(common::Pow2(delta[0]) + common::Pow2(delta[1]));
    const T sin_delta_over_delta =
        delta_norm < 1e-6 ? T(1.) : ceres::sin(delta_norm) / delta_norm;
    T q_delta[4];
    q_delta[0] = delta_norm < 1e-6 ? T(1.) : ceres::cos(delta_norm);
    q_delta[1] = sin_delta_over_delta * delta[0];
    q_delta[2] = sin_delta_over_delta * delta[1];
    q_delta[3] = T(0.);
    // We apply the 'delta' which is interpreted as an angle-axis rotation
    // vector in the xy-plane of the submap frame. This way we can align to
    // gravity because rotations around the z-axis in the submap frame do not
    // change gravity alignment, while disallowing random rotations of the map
    // that have nothing to do with gravity alignment (i.e. we disallow steps
    // just changing "yaw" of the complete map).
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }
};

}  // namespace

OptimizationProblem::OptimizationProblem(
    const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
        options,
    FixZ fix_z)
    : options_(options), fix_z_(fix_z) {}

OptimizationProblem::~OptimizationProblem() {}

void OptimizationProblem::AddImuData(const int trajectory_id,
                                     const common::Time time,
                                     const Eigen::Vector3d& linear_acceleration,
                                     const Eigen::Vector3d& angular_velocity) {
  CHECK_GE(trajectory_id, 0);
  imu_data_.resize(
      std::max(submap_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  imu_data_[trajectory_id].push_back(
      ImuData{time, linear_acceleration, angular_velocity});
}

void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid3d& point_cloud_pose) {
  node_data_.push_back(NodeData{trajectory_id, time, point_cloud_pose});
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

void OptimizationProblem::Solve(const std::vector<Constraint>& constraints) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  // Compute the indices of consecutive nodes for each trajectory.
  std::vector<std::vector<size_t>> nodes_per_trajectory(submap_data_.size());
  for (size_t j = 0; j != node_data_.size(); ++j) {
    nodes_per_trajectory.at(node_data_[j].trajectory_id).push_back(j);
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
  std::deque<CeresPose> C_point_clouds;
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
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
    }
  }
  for (size_t j = 0; j != node_data_.size(); ++j) {
    C_point_clouds.emplace_back(
        node_data_[j].point_cloud_pose, translation_parameterization(),
        common::make_unique<ceres::QuaternionParameterization>(), &problem);
  }

  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    CHECK_GE(constraint.j, 0);
    CHECK_LT(constraint.j, node_data_.size());
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
        C_point_clouds[constraint.j].rotation(),
        C_point_clouds[constraint.j].translation());
  }

  // Add constraints based on IMU observations of angular velocities and
  // linear acceleration.
  for (size_t trajectory_id = 0; trajectory_id != nodes_per_trajectory.size();
       ++trajectory_id) {
    const std::vector<size_t>& node_indices =
        nodes_per_trajectory[trajectory_id];
    const std::deque<ImuData>& imu_data = imu_data_.at(trajectory_id);
    CHECK(!node_indices.empty());
    CHECK(!imu_data.empty());

    // Skip IMU data before the first node of this trajectory.
    auto it = imu_data.cbegin();
    while ((it + 1) != imu_data.cend() &&
           (it + 1)->time <= node_data_[node_indices[0]].time) {
      ++it;
    }

    for (size_t j = 1; j < node_indices.size(); ++j) {
      auto it2 = it;
      const IntegrateImuResult<double> result =
          IntegrateImu(imu_data, node_data_[node_indices[j - 1]].time,
                       node_data_[node_indices[j]].time, &it);
      if (j + 1 < node_indices.size()) {
        const common::Time first_time = node_data_[node_indices[j - 1]].time;
        const common::Time second_time = node_data_[node_indices[j]].time;
        const common::Time third_time = node_data_[node_indices[j + 1]].time;
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
                                            3, 3, 1>(
                new AccelerationCostFunction(
                    options_.acceleration_weight(), delta_velocity,
                    common::ToSeconds(first_duration),
                    common::ToSeconds(second_duration))),
            nullptr, C_point_clouds[node_indices[j]].rotation(),
            C_point_clouds[node_indices[j - 1]].translation(),
            C_point_clouds[node_indices[j]].translation(),
            C_point_clouds[node_indices[j + 1]].translation(),
            &gravity_constant_);
      }
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<RotationCostFunction, 3, 4, 4>(
              new RotationCostFunction(options_.rotation_weight(),
                                       result.delta_rotation)),
          nullptr, C_point_clouds[node_indices[j - 1]].rotation(),
          C_point_clouds[node_indices[j]].rotation());
    }
  }

  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
    LOG(INFO) << "Gravity was: " << gravity_constant_;
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

  for (size_t j = 0; j != node_data_.size(); ++j) {
    node_data_[j].point_cloud_pose = C_point_clouds[j].ToRigid();
  }
}

const std::vector<NodeData>& OptimizationProblem::node_data() const {
  return node_data_;
}

const std::vector<std::vector<SubmapData>>& OptimizationProblem::submap_data()
    const {
  return submap_data_;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_3d
}  // namespace cartographer
