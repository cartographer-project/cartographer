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

#include "cartographer/mapping_3d/pose_graph/optimization_problem.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
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
#include "cartographer/mapping_3d/pose_graph/spa_cost_function.h"
#include "cartographer/mapping_3d/rotation_cost_function.h"
#include "cartographer/mapping_3d/rotation_parameterization.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace pose_graph {

namespace {

// For odometry and fixed frame pose.
template <typename MapByTimeType>
std::unique_ptr<transform::Rigid3d> Interpolate(
    const MapByTimeType& map_by_time, const int trajectory_id,
    const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
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

}  // namespace

OptimizationProblem::OptimizationProblem(
    const mapping::pose_graph::proto::OptimizationProblemOptions& options,
    FixZ fix_z)
    : options_(options), fix_z_(fix_z) {}

OptimizationProblem::~OptimizationProblem() {}

void OptimizationProblem::AddImuData(const int trajectory_id,
                                     const sensor::ImuData& imu_data) {
  imu_data_.Append(trajectory_id, imu_data);
}

void OptimizationProblem::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void OptimizationProblem::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}

void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid3d& local_pose,
    const transform::Rigid3d& global_pose) {
  CHECK_GE(trajectory_id, 0);
  trajectory_data_.resize(std::max(trajectory_data_.size(),
                                   static_cast<size_t>(trajectory_id) + 1));
  node_data_.Append(trajectory_id, NodeData{time, local_pose, global_pose});
}

void OptimizationProblem::InsertTrajectoryNode(
    const mapping::NodeId& node_id, const common::Time time,
    const transform::Rigid3d& local_pose,
    const transform::Rigid3d& global_pose) {
  CHECK_GE(node_id.trajectory_id, 0);
  trajectory_data_.resize(std::max(
      trajectory_data_.size(), static_cast<size_t>(node_id.trajectory_id) + 1));
  node_data_.Insert(node_id, NodeData{time, local_pose, global_pose});
}

void OptimizationProblem::TrimTrajectoryNode(const mapping::NodeId& node_id) {
  imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
}

void OptimizationProblem::AddSubmap(
    const int trajectory_id, const transform::Rigid3d& global_submap_pose) {
  CHECK_GE(trajectory_id, 0);
  trajectory_data_.resize(std::max(trajectory_data_.size(),
                                   static_cast<size_t>(trajectory_id) + 1));
  submap_data_.Append(trajectory_id, SubmapData{global_submap_pose});
}

void OptimizationProblem::InsertSubmap(
    const mapping::SubmapId& submap_id,
    const transform::Rigid3d& global_submap_pose) {
  CHECK_GE(submap_id.trajectory_id, 0);
  trajectory_data_.resize(
      std::max(trajectory_data_.size(),
               static_cast<size_t>(submap_id.trajectory_id) + 1));
  submap_data_.Insert(submap_id, SubmapData{global_submap_pose});
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

  const auto translation_parameterization =
      [this]() -> std::unique_ptr<ceres::LocalParameterization> {
    return fix_z_ == FixZ::kYes
               ? common::make_unique<ceres::SubsetParameterization>(
                     3, std::vector<int>{2})
               : nullptr;
  };

  // Set the starting point.
  CHECK(!submap_data_.empty());
  CHECK(submap_data_.Contains(mapping::SubmapId{0, 0}));
  mapping::MapById<mapping::SubmapId, CeresPose> C_submaps;
  mapping::MapById<mapping::NodeId, CeresPose> C_nodes;
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    if (first_submap) {
      first_submap = false;
      // Fix the first submap of the first trajectory except for allowing
      // gravity alignment.
      C_submaps.Insert(
          submap_id_data.id,
          CeresPose(submap_id_data.data.global_pose,
                    translation_parameterization(),
                    common::make_unique<ceres::AutoDiffLocalParameterization<
                        ConstantYawQuaternionPlus, 4, 2>>(),
                    &problem));
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    } else {
      C_submaps.Insert(
          submap_id_data.id,
          CeresPose(submap_id_data.data.global_pose,
                    translation_parameterization(),
                    common::make_unique<ceres::QuaternionParameterization>(),
                    &problem));
    }
    if (frozen) {
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    }
  }
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(
        node_id_data.id,
        CeresPose(node_id_data.data.global_pose, translation_parameterization(),
                  common::make_unique<ceres::QuaternionParameterization>(),
                  &problem));
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_nodes.at(node_id_data.id).translation());
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
        C_submaps.at(constraint.submap_id).rotation(),
        C_submaps.at(constraint.submap_id).translation(),
        C_nodes.at(constraint.node_id).rotation(),
        C_nodes.at(constraint.node_id).translation());
  }

  // Add constraints based on IMU observations of angular velocities and
  // linear acceleration.
  if (fix_z_ == FixZ::kNo) {
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
      const int trajectory_id = node_it->id.trajectory_id;
      const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
      if (frozen_trajectories.count(trajectory_id) != 0) {
        // We skip frozen trajectories.
        node_it = trajectory_end;
        continue;
      }
      TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);

      problem.AddParameterBlock(trajectory_data.imu_calibration.data(), 4,
                                new ceres::QuaternionParameterization());
      CHECK(imu_data_.HasTrajectory(trajectory_id));
      const auto imu_data = imu_data_.trajectory(trajectory_id);
      CHECK(imu_data.begin() != imu_data.end());

      auto imu_it = imu_data.begin();
      auto prev_node_it = node_it;
      for (++node_it; node_it != trajectory_end; ++node_it) {
        const mapping::NodeId first_node_id = prev_node_it->id;
        const NodeData& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const mapping::NodeId second_node_id = node_it->id;
        const NodeData& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        // Skip IMU data before the node.
        while (std::next(imu_it) != imu_data.end() &&
               std::next(imu_it)->time <= first_node_data.time) {
          ++imu_it;
        }

        auto imu_it2 = imu_it;
        const IntegrateImuResult<double> result = IntegrateImu(
            imu_data, first_node_data.time, second_node_data.time, &imu_it);
        const auto next_node_it = std::next(node_it);
        if (next_node_it != trajectory_end &&
            next_node_it->id.node_index == second_node_id.node_index + 1) {
          const mapping::NodeId third_node_id = next_node_it->id;
          const NodeData& third_node_data = next_node_it->data;
          const common::Time first_time = first_node_data.time;
          const common::Time second_time = second_node_data.time;
          const common::Time third_time = third_node_data.time;
          const common::Duration first_duration = second_time - first_time;
          const common::Duration second_duration = third_time - second_time;
          const common::Time first_center = first_time + first_duration / 2;
          const common::Time second_center = second_time + second_duration / 2;
          const IntegrateImuResult<double> result_to_first_center =
              IntegrateImu(imu_data, first_time, first_center, &imu_it2);
          const IntegrateImuResult<double> result_center_to_center =
              IntegrateImu(imu_data, first_center, second_center, &imu_it2);
          // 'delta_velocity' is the change in velocity from the point in time
          // halfway between the first and second poses to halfway between
          // second and third pose. It is computed from IMU data and still
          // contains a delta due to gravity. The orientation of this vector is
          // in the IMU frame at the second pose.
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
              nullptr, C_nodes.at(second_node_id).rotation(),
              C_nodes.at(first_node_id).translation(),
              C_nodes.at(second_node_id).translation(),
              C_nodes.at(third_node_id).translation(),
              &trajectory_data.gravity_constant,
              trajectory_data.imu_calibration.data());
        }
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RotationCostFunction, 3, 4, 4, 4>(
                new RotationCostFunction(options_.rotation_weight(),
                                         result.delta_rotation)),
            nullptr, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(second_node_id).rotation(),
            trajectory_data.imu_calibration.data());
      }
    }
  }

  if (fix_z_ == FixZ::kYes) {
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
        const mapping::NodeId first_node_id = prev_node_it->id;
        const NodeData& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const mapping::NodeId second_node_id = node_it->id;
        const NodeData& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        const transform::Rigid3d relative_pose = ComputeRelativePose(
            trajectory_id, first_node_data, second_node_data);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SpaCostFunction, 6, 4, 3, 4, 3>(
                new SpaCostFunction(Constraint::Pose{
                    relative_pose,
                    options_.consecutive_node_translation_weight(),
                    options_.consecutive_node_rotation_weight()})),
            nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(first_node_id).translation(),
            C_nodes.at(second_node_id).rotation(),
            C_nodes.at(second_node_id).translation());
      }
    }
  }

  // Add fixed frame pose constraints.
  std::deque<CeresPose> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    bool fixed_frame_pose_initialized = false;
    for (; node_it != trajectory_end; ++node_it) {
      const mapping::NodeId node_id = node_it->id;
      const NodeData& node_data = node_it->data;

      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const mapping::PoseGraph::Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        const transform::Rigid3d fixed_frame_pose_in_map =
            node_data.global_pose * constraint_pose.zbar_ij.inverse();
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
          C_fixed_frames.back().translation(), C_nodes.at(node_id).rotation(),
          C_nodes.at(node_id).translation());
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
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        C_submap_id_data.data.ToRigid();
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose =
        C_node_id_data.data.ToRigid();
  }
}

const mapping::MapById<mapping::NodeId, NodeData>&
OptimizationProblem::node_data() const {
  return node_data_;
}

const mapping::MapById<mapping::SubmapId, SubmapData>&
OptimizationProblem::submap_data() const {
  return submap_data_;
}

const sensor::MapByTime<sensor::ImuData>& OptimizationProblem::imu_data()
    const {
  return imu_data_;
}

const sensor::MapByTime<sensor::OdometryData>&
OptimizationProblem::odometry_data() const {
  return odometry_data_;
}

transform::Rigid3d OptimizationProblem::ComputeRelativePose(
    const int trajectory_id, const NodeData& first_node_data,
    const NodeData& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        Interpolate(odometry_data_, trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        Interpolate(odometry_data_, trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      return first_node_odometry->inverse() * (*second_node_odometry);
    }
  }
  return first_node_data.local_pose.inverse() * second_node_data.local_pose;
}

}  // namespace pose_graph
}  // namespace mapping_3d
}  // namespace cartographer
