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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_SPA_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_SPA_COST_FUNCTION_2D_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph/cost_helpers.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace pose_graph {

class SpaCostFunction2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const PoseGraph::Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                           3 /* pose variables */,
                                           3 /* pose variables */>(
        new SpaCostFunction2D(pose));
  }

  template <typename T>
  bool operator()(const T* const c_i, const T* const c_j, T* e) const {
    using pose_graph::ComputeUnscaledError;
    using pose_graph::ScaleError;

    const std::array<T, 3> error = ScaleError(
        ComputeUnscaledError(transform::Project2D(pose_.zbar_ij), c_i, c_j),
        T(pose_.translation_weight), T(pose_.rotation_weight));
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit SpaCostFunction2D(const PoseGraph::Constraint::Pose& pose)
      : pose_(pose) {}

  const PoseGraph::Constraint::Pose pose_;
};

}  // namespace pose_graph
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_POSE_GRAPH_SPA_COST_FUNCTION_2D_H_
