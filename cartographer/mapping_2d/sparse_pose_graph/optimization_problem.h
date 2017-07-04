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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_

#include <array>
#include <deque>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/mapping_3d/imu_integration.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

struct NodeData {
  common::Time time;
  transform::Rigid2d initial_point_cloud_pose;
  transform::Rigid2d point_cloud_pose;
};

struct SubmapData {
  transform::Rigid2d pose;
};

// Implements the SPA loop closure method.
class OptimizationProblem {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;

  explicit OptimizationProblem(
      const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
          options);
  ~OptimizationProblem();

  OptimizationProblem(const OptimizationProblem&) = delete;
  OptimizationProblem& operator=(const OptimizationProblem&) = delete;

  void AddImuData(int trajectory_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  void AddTrajectoryNode(int trajectory_id, common::Time time,
                         const transform::Rigid2d& initial_point_cloud_pose,
                         const transform::Rigid2d& point_cloud_pose);
  void TrimTrajectoryNode(const mapping::NodeId& node_id);
  void AddSubmap(int trajectory_id, const transform::Rigid2d& submap_pose);
  void TrimSubmap(const mapping::SubmapId& submap_id);

  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses.
  void Solve(const std::vector<Constraint>& constraints,
             const std::set<int>& frozen_trajectories);

  const std::vector<std::deque<NodeData>>& node_data() const;
  const std::vector<std::deque<SubmapData>>& submap_data() const;

  int num_trimmed_nodes(int trajectory_id) const;
  int num_trimmed_submaps(int trajectory_id) const;

 private:
  struct TrajectoryData {
    // TODO(hrapp): Remove, once we can relabel constraints.
    int num_trimmed_nodes = 0;
    int num_trimmed_submaps = 0;
  };
  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
  std::vector<std::deque<mapping_3d::ImuData>> imu_data_;
  std::vector<std::deque<NodeData>> node_data_;
  std::vector<std::deque<SubmapData>> submap_data_;
  std::vector<TrajectoryData> trajectory_data_;
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
