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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_PARAMETERIZATION_PARAMETERIZATION_H_
#define CARTOGRAPHER_POSE_GRAPH_NODE_PARAMETERIZATION_PARAMETERIZATION_H_

#include "cartographer/pose_graph/proto/node.pb.h"
#include "ceres/local_parameterization.h"

namespace cartographer {
namespace pose_graph {

class Parameterization {
 public:
  explicit Parameterization(const proto::Parameterization& proto);

  const proto::Parameterization& ToProto() const { return proto_; }

  ceres::LocalParameterization* ceres_parameterization() const {
    return ceres_parameterization_.get();
  }

 private:
  const proto::Parameterization proto_;
  const std::unique_ptr<ceres::LocalParameterization> ceres_parameterization_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_PARAMETERIZATION_PARAMETERIZATION_H_
