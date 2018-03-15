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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_OPTIMIZATION_PROBLEM_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_OPTIMIZATION_PROBLEM_3D_H_

#include <array>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/optional.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/pose_graph/optimization_problem_interface.h"
#include "cartographer/mapping/pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

struct NodeData3D {
  common::Time time;
  transform::Rigid3d local_pose;
  transform::Rigid3d global_pose;
};

struct SubmapData3D {
  transform::Rigid3d global_pose;
};

class OptimizationProblem3D
    : public OptimizationProblemInterface<NodeData3D, SubmapData3D,
                                          transform::Rigid3d> {
 public:
  explicit OptimizationProblem3D(
      const pose_graph::proto::OptimizationProblemOptions& options);
  ~OptimizationProblem3D();

  OptimizationProblem3D(const OptimizationProblem3D&) = delete;
  OptimizationProblem3D& operator=(const OptimizationProblem3D&) = delete;

  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override;
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override;
  void AddTrajectoryNode(int trajectory_id,
                         const NodeData3D& node_data) override;
  void InsertTrajectoryNode(const NodeId& node_id,
                            const NodeData3D& node_data) override;
  void TrimTrajectoryNode(const NodeId& node_id) override;
  void AddSubmap(int trajectory_id,
                 const transform::Rigid3d& global_submap_pose) override;
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid3d& global_submap_pose) override;
  void TrimSubmap(const SubmapId& submap_id) override;
  void SetMaxNumIterations(int32 max_num_iterations) override;

  void Solve(
      const std::vector<Constraint>& constraints,
      const std::set<int>& frozen_trajectories,
      const std::map<std::string, LandmarkNode>& landmark_nodes) override;

  const MapById<NodeId, NodeData3D>& node_data() const override {
    return node_data_;
  }
  const MapById<SubmapId, SubmapData3D>& submap_data() const override {
    return submap_data_;
  }
  const std::map<std::string, transform::Rigid3d>& landmark_data()
      const override {
    return landmark_data_;
  }
  const sensor::MapByTime<sensor::ImuData>& imu_data() const override {
    return imu_data_;
  }
  const sensor::MapByTime<sensor::OdometryData>& odometry_data()
      const override {
    return odometry_data_;
  }

  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data);
  void SetTrajectoryData(
      int trajectory_id,
      const PoseGraphInterface::TrajectoryData& trajectory_data);
  const sensor::MapByTime<sensor::FixedFramePoseData>& fixed_frame_pose_data()
      const {
    return fixed_frame_pose_data_;
  }
  const std::map<int, PoseGraphInterface::TrajectoryData>& trajectory_data()
      const {
    return trajectory_data_;
  }

 private:
  // Uses odometry if available, otherwise the local SLAM results.
  transform::Rigid3d ComputeRelativePose(
      int trajectory_id, const NodeData3D& first_node_data,
      const NodeData3D& second_node_data) const;

  pose_graph::proto::OptimizationProblemOptions options_;
  MapById<NodeId, NodeData3D> node_data_;
  MapById<SubmapId, SubmapData3D> submap_data_;
  std::map<std::string, transform::Rigid3d> landmark_data_;
  sensor::MapByTime<sensor::ImuData> imu_data_;
  sensor::MapByTime<sensor::OdometryData> odometry_data_;
  sensor::MapByTime<sensor::FixedFramePoseData> fixed_frame_pose_data_;
  std::map<int, PoseGraphInterface::TrajectoryData> trajectory_data_;
};

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_OPTIMIZATION_PROBLEM_3D_H_
