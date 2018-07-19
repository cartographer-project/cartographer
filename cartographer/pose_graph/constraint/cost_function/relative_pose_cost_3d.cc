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

#include "cartographer/pose_graph/constraint/cost_function/relative_pose_cost_3d.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/internal/optimization/cost_functions/cost_helpers.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace pose_graph {
namespace {

class AutoDiffCostFunction3D {
 public:
  explicit AutoDiffCostFunction3D(const RelativePoseCost3D* relative_pose_cost)
      : relative_pose_cost_(relative_pose_cost) {}

  template <typename T>
  bool operator()(const T* const c_i_translation, const T* const c_i_rotation,
                  const T* const c_j_translation, const T* const c_j_rotation,
                  T* const error_out) const {
    const std::array<T, 6> error = mapping::optimization::ScaleError(
        mapping::optimization::ComputeUnscaledError(
            relative_pose_cost_->GetFirstTSecond(), c_i_rotation,
            c_i_translation, c_j_rotation, c_j_translation),
        relative_pose_cost_->GetTranslationWeight(),
        relative_pose_cost_->GetRotationWeight());
    std::copy(std::begin(error), std::end(error), error_out);
    return true;
  }

 private:
  const RelativePoseCost3D* relative_pose_cost_;
};

using AutoDiffFunctor =
    ceres::AutoDiffCostFunction<AutoDiffCostFunction3D,
                                6 /* number of residuals */,
                                3 /* translation variables first pose */,
                                4 /* rotation variables first pose*/,
                                3 /* translation variables second pose */,
                                4 /* rotation variables second pose*/>;

}  // namespace

RelativePoseCost3D::RelativePoseCost3D(
    const proto::RelativePose3D::Parameters& parameters)
    : translation_weight_(parameters.translation_weight()),
      rotation_weight_(parameters.rotation_weight()),
      first_T_second_(transform::ToRigid3(parameters.first_t_second())),
      function_(common::make_unique<AutoDiffFunctor>(
          new AutoDiffCostFunction3D(this))) {}

proto::RelativePose3D::Parameters RelativePoseCost3D::ToProto() const {
  proto::RelativePose3D::Parameters parameters;
  parameters.set_translation_weight(translation_weight_);
  parameters.set_rotation_weight(rotation_weight_);
  *parameters.mutable_first_t_second() = transform::ToProto(first_T_second_);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer