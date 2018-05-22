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

#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(
      const PoseGraphInterface::Constraint::Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error =
        ScaleError(ComputeUnscaledError(
                       transform::Project2D(observed_relative_pose_.zbar_ij),
                       start_pose, end_pose),
                   observed_relative_pose_.translation_weight,
                   observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const PoseGraphInterface::Constraint::Pose observed_relative_pose_;
};

class AnalyticalSpaCostFunction2D
    : public ceres::SizedCostFunction<3 /* number of residuals */,
                                      3 /* size of start pose */,
                                      3 /* size of end pose */> {
 public:
  explicit AnalyticalSpaCostFunction2D(
      const PoseGraphInterface::Constraint::Pose& constraint_pose)
      : observed_relative_pose_(transform::Project2D(constraint_pose.zbar_ij)),
        translation_weight_(constraint_pose.translation_weight),
        rotation_weight_(constraint_pose.rotation_weight) {}
  virtual ~AnalyticalSpaCostFunction2D() {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    double const* start = parameters[0];
    double const* end = parameters[1];

    const double cos_start_rotation = cos(start[2]);
    const double sin_start_rotation = sin(start[2]);
    const double delta_x = end[0] - start[0];
    const double delta_y = end[1] - start[1];

    residuals[0] =
        translation_weight_ *
        (observed_relative_pose_.translation().x() -
         (cos_start_rotation * delta_x + sin_start_rotation * delta_y));
    residuals[1] =
        translation_weight_ *
        (observed_relative_pose_.translation().y() -
         (-sin_start_rotation * delta_x + cos_start_rotation * delta_y));
    residuals[2] =
        rotation_weight_ *
        common::NormalizeAngleDifference(
            observed_relative_pose_.rotation().angle() - (end[2] - start[2]));
    if (jacobians == NULL) return true;

    const double weighted_cos_start_rotation =
        translation_weight_ * cos_start_rotation;
    const double weighted_sin_start_rotation =
        translation_weight_ * sin_start_rotation;

    // Jacobians in Ceres are ordered by the parameter blocks:
    // jacobian[i] = [(dr_0 / dx_i)^T, ..., (dr_n / dx_i)^T].
    if (jacobians[0] != NULL) {
      jacobians[0][0] = weighted_cos_start_rotation;
      jacobians[0][1] = weighted_sin_start_rotation;
      jacobians[0][2] = weighted_sin_start_rotation * delta_x -
                        weighted_cos_start_rotation * delta_y;
      jacobians[0][3] = -weighted_sin_start_rotation;
      jacobians[0][4] = weighted_cos_start_rotation;
      jacobians[0][5] = weighted_cos_start_rotation * delta_x +
                        weighted_sin_start_rotation * delta_y;
      jacobians[0][6] = 0;
      jacobians[0][7] = 0;
      jacobians[0][8] = rotation_weight_;
    }
    if (jacobians[1] != NULL) {
      jacobians[1][0] = -weighted_cos_start_rotation;
      jacobians[1][1] = -weighted_sin_start_rotation;
      jacobians[1][2] = 0;
      jacobians[1][3] = weighted_sin_start_rotation;
      jacobians[1][4] = -weighted_cos_start_rotation;
      jacobians[1][5] = 0;
      jacobians[1][6] = 0;
      jacobians[1][7] = 0;
      jacobians[1][8] = -rotation_weight_;
    }
    return true;
  }

 private:
  const transform::Rigid2d observed_relative_pose_;
  const double translation_weight_;
  const double rotation_weight_;
};

}  // namespace

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const PoseGraphInterface::Constraint::Pose& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

ceres::CostFunction* CreateAnalyticalSpaCostFunction(
    const PoseGraphInterface::Constraint::Pose& observed_relative_pose) {
  return new AnalyticalSpaCostFunction2D(observed_relative_pose);
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
