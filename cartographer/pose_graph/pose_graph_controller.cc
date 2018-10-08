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

#include "cartographer/pose_graph/pose_graph_controller.h"

namespace cartographer {
namespace pose_graph {

void PoseGraphController::AddNode(const proto::Node& node) {
  absl::MutexLock locker(&mutex_);
  AddNodeToPoseGraphData(node, &data_);
}

void PoseGraphController::AddConstraint(const proto::Constraint& constraint) {
  absl::MutexLock locker(&mutex_);
  AddConstraintToPoseGraphData(constraint, &data_);
}

Solver::SolverStatus PoseGraphController::Optimize() {
  absl::MutexLock locker(&mutex_);
  return solver_->Solve(&data_);
}

}  // namespace pose_graph
}  // namespace cartographer
