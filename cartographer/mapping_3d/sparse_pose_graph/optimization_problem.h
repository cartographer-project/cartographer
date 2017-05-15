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

#ifndef CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
#define CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_

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
namespace mapping_3d {
namespace sparse_pose_graph {

struct NodeData {
  common::Time time;
  transform::Rigid3d point_cloud_pose;
};

struct SubmapData {
  transform::Rigid3d pose;
};

// Implements the SPA loop closure method.
class OptimizationProblem {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;

  enum class FixZ { kYes, kNo };

  OptimizationProblem(
      const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
          options,
      FixZ fix_z);
  ~OptimizationProblem();

  OptimizationProblem(const OptimizationProblem&) = delete;
  OptimizationProblem& operator=(const OptimizationProblem&) = delete;

  void AddImuData(int trajectory_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  void AddTrajectoryNode(int trajectory_id, common::Time time,
                         const transform::Rigid3d& point_cloud_pose);
  void AddSubmap(int trajectory_id, const transform::Rigid3d& submap_pose);

  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses.
  void Solve(const std::vector<Constraint>& constraints);

  const std::vector<std::vector<NodeData>>& node_data() const;
  const std::vector<std::vector<SubmapData>>& submap_data() const;

 private:
  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
  FixZ fix_z_;
  std::vector<std::deque<ImuData>> imu_data_;
  std::vector<std::vector<NodeData>> node_data_;
  std::vector<std::vector<SubmapData>> submap_data_;
  double gravity_constant_ = 9.8;
};

}  // namespace sparse_pose_graph
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
