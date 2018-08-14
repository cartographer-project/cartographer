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

#include "cartographer/pose_graph/node/parameterization/parameterization.h"

#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/3d/rotation_parameterization.h"
#include "ceres/autodiff_local_parameterization.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace pose_graph {
namespace {

using absl::make_unique;
using ceres::AutoDiffLocalParameterization;
using ceres::LocalParameterization;

// TODO(pifon): Check if the functors are compatible with our quaternions. Test!
std::unique_ptr<LocalParameterization> CeresParameterizationFromProto(
    const proto::Parameterization& parameterization) {
  switch (parameterization) {
    case (proto::Parameterization::NONE):
      return nullptr;
    case (proto::Parameterization::YAW_ONLY):
      return make_unique<AutoDiffLocalParameterization<
          mapping::YawOnlyQuaternionPlus, 4, 1>>();
    case (proto::Parameterization::YAW_CONSTANT):
      return make_unique<AutoDiffLocalParameterization<
          mapping::ConstantYawQuaternionPlus, 4, 1>>();
    case (proto::Parameterization::FIX_Z):
      return make_unique<ceres::SubsetParameterization>(
          3, /* constant parameters */ std::vector<int>{2});
    default:
      LOG(FATAL) << "Parameterization is not known.";
  }
  return nullptr;
}

}  // namespace

Parameterization::Parameterization(const proto::Parameterization& proto)
    : proto_(proto),
      ceres_parameterization_(CeresParameterizationFromProto(proto_)) {}

}  // namespace pose_graph
}  // namespace cartographer
