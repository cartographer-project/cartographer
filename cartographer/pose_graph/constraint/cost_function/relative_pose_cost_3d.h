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

#ifndef CARTOGRAPHER_RELATIVE_POSE_COST_3D_H
#define CARTOGRAPHER_RELATIVE_POSE_COST_3D_H

#include "cartographer/pose_graph/proto/cost_function.pb.h"
#include "cartographer/transform/transform.h"
#include "ceres/sized_cost_function.h"

namespace cartographer {
namespace pose_graph {

class RelativePoseCost3D {
 public:
  using CeresFunction =
      ceres::SizedCostFunction<6 /* number of residuals */,
                               3 /* translation variables first pose */,
                               4 /* rotation variables first pose*/,
                               3 /* translation variables second pose */,
                               4 /* rotation variables second pose*/>;

  explicit RelativePoseCost3D(
      const proto::RelativePose3D::Parameters& parameters);

  const CeresFunction* GetCeresFunction() { return function_.get(); }

  proto::RelativePose3D::Parameters ToProto() const;

  double GetTranslationWeight() const { return translation_weight_; }
  double GetRotationWeight() const { return rotation_weight_; }
  transform::Rigid3d GetFirstTSecond() const { return first_T_second_; }

 private:
  const double translation_weight_;
  const double rotation_weight_;
  const transform::Rigid3d first_T_second_;
  const std::unique_ptr<CeresFunction> function_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_RELATIVE_POSE_COST_3D_H
