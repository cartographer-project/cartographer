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

#include "cartographer/pose_graph/constraint/loss_function/loss_function.h"

#include "absl/memory/memory.h"

namespace cartographer {
namespace pose_graph {
namespace {

std::unique_ptr<ceres::LossFunction> CeresLossFromProto(
    const proto::LossFunction& proto) {
  switch (proto.Type_case()) {
    case proto::LossFunction::kHuberLoss:
      return absl::make_unique<ceres::HuberLoss>(proto.huber_loss().scale());
    case proto::LossFunction::kQuadraticLoss:
      return nullptr;
    default:
      LOG(FATAL) << "The loss function is not specified.";
      return nullptr;
  }
}

}  // namespace

LossFunction::LossFunction(const proto::LossFunction& proto)
    : proto_(proto), ceres_loss_(CeresLossFromProto(proto_)) {}

}  // namespace pose_graph
}  // namespace cartographer
