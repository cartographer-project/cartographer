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

#ifndef CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_LOSS_FUNCTION_H_
#define CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_LOSS_FUNCTION_H_

#include <memory>

#include "cartographer/pose_graph/proto/loss_function.pb.h"
#include "ceres/loss_function.h"

namespace cartographer {
namespace pose_graph {

class LossFunction {
 public:
  explicit LossFunction(const proto::LossFunction& proto);

  const proto::LossFunction& ToProto() const { return proto_; }

  ceres::LossFunction* ceres_loss() const { return ceres_loss_.get(); }

 private:
  const proto::LossFunction proto_;
  const std::unique_ptr<ceres::LossFunction> ceres_loss_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_CONSTRAINT_LOSS_FUNCTION_H_
