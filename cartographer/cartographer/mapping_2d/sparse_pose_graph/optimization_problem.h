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
#include "cartographer/common/lua_parameter_dictionary.h"
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
  using Constraint = mapping::SparsePoseGraph::Constraint2D;

  class SpaCostFunction {
   public:
    explicit SpaCostFunction(const Constraint::Pose& pose) : pose_(pose) {}

    // Computes the error in the proposed change to the scan-to-submap alignment
    // 'zbar_ij' where submap pose 'c_i' and scan pose 'c_j' are both in an
    // arbitrary common frame.
    template <typename T>
    static std::array<T, 3> ComputeUnscaledError(
        const transform::Rigid2d& zbar_ij, const T* const c_i,
        const T* const c_j) {
      const T cos_theta_i = cos(c_i[2]);
      const T sin_theta_i = sin(c_i[2]);
      const T delta_x = c_j[0] - c_i[0];
      const T delta_y = c_j[1] - c_i[1];
      const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                      -sin_theta_i * delta_x + cos_theta_i * delta_y,
                      c_j[2] - c_i[2]};
      return {{T(zbar_ij.translation().x()) - h[0],
               T(zbar_ij.translation().y()) - h[1],
               common::NormalizeAngleDifference(T(zbar_ij.rotation().angle()) -
                                                h[2])}};
    }

    // Scales the result of ComputeUnscaledError scaled by 'sqrt_Lambda_ij' and
    // stores it in 'e'.
    template <typename T>
    static void ComputeScaledError(const Constraint::Pose& pose,
                                   const T* const c_i, const T* const c_j,
                                   T* const e) {
      std::array<T, 3> e_ij = ComputeUnscaledError(pose.zbar_ij, c_i, c_j);
      (Eigen::Map<Eigen::Matrix<T, 3, 1>>(e)) =
          pose.sqrt_Lambda_ij.cast<T>() *
          Eigen::Map<Eigen::Matrix<T, 3, 1>>(e_ij.data());
    }

    template <typename T>
    bool operator()(const T* const c_i, const T* const c_j, T* e) const {
      ComputeScaledError(pose_, c_i, c_j, e);
      return true;
    }

   private:
    const Constraint::Pose pose_;
  };

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
  class TieToPoseCostFunction {
   public:
    explicit TieToPoseCostFunction(const Constraint::Pose& pose)
        : pose_(pose) {}

    template <typename T>
    bool operator()(const T* const c_j, T* e) const;

   private:
    const Constraint::Pose pose_;
  };

  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
