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
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/mapping_2d/submaps.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

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

  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses. The point cloud at 'point_cloud_poses[i]'
  // belongs to 'trajectories[i]'. Within a given trajectory, scans are expected
  // to be contiguous.
  void Solve(const std::vector<Constraint>& constraints,
             const std::vector<const mapping::Submaps*>& trajectories,
             const std::vector<transform::Rigid2d>& initial_point_cloud_poses,
             std::vector<transform::Rigid2d>* point_cloud_poses,
             std::vector<transform::Rigid2d>* submap_transforms);

 private:
  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
