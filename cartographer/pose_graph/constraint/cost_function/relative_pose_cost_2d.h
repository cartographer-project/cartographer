/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_RELATIVE_POSE_COST_2D_H_
#define CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_RELATIVE_POSE_COST_2D_H_

#include "cartographer/pose_graph/proto/cost_function.pb.h"
#include "cartographer/transform/transform.h"
#include "ceres/sized_cost_function.h"

namespace cartographer {
namespace pose_graph {

class RelativePoseCost2D
    : public ceres::SizedCostFunction<3 /* number of residuals */,
                                      3 /* size of first pose */,
                                      3 /* size of second pose */> {
 public:
  explicit RelativePoseCost2D(
      const proto::RelativePose2D::Parameters& parameters);

  proto::RelativePose2D::Parameters ToProto() const;

  // Parameters are packed as [first_pose_2d, second_pose_2d], where each 2D
  // pose is [translation_x, translation_y, rotation].
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const final;

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const transform::Rigid2d first_T_second_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_COST_FUNCTION_RELATIVE_POSE_COST_2D_H_
