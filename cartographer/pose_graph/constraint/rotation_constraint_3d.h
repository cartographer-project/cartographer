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

#ifndef CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_ROTATION_CONSTRAINT_3D_H_
#define CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_ROTATION_CONSTRAINT_3D_H_

#include "cartographer/pose_graph/constraint/constraint.h"
#include "cartographer/pose_graph/constraint/cost_function/rotation_cost_3d.h"

namespace cartographer {
namespace pose_graph {

class RotationContraint3D : public Constraint {
 public:
  RotationContraint3D(const ConstraintId& id,
                      const proto::LossFunction& loss_function_proto,
                      const proto::Rotation3D& proto);

  void AddToOptimizer(Nodes* nodes, ceres::Problem* problem) const final;

 protected:
  proto::CostFunction ToCostFunctionProto() const final;

 private:
  using AutoDiffFunction = ceres::AutoDiffCostFunction<
      RotationCost3D, 3 /* number of residuals */, 4 /* rotation first pose */,
      4 /* rotation second pose */, 4 /* imu calibration */>;
  NodeId first_;
  NodeId second_;
  NodeId imu_calibration_;
  // The cost function is owned by the ceres cost function.
  RotationCost3D* const cost_;
  std::unique_ptr<AutoDiffFunction> ceres_cost_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_ROTATION_CONSTRAINT_3D_H_
