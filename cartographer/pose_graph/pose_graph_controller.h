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

#ifndef CARTOGRAPHER_POSE_GRAPH_POSE_GRAPH_CONTROLLER_H_
#define CARTOGRAPHER_POSE_GRAPH_POSE_GRAPH_CONTROLLER_H_

#include "absl/synchronization/mutex.h"
#include "cartographer/pose_graph/pose_graph_data.h"
#include "cartographer/pose_graph/solver/solver.h"

namespace cartographer {
namespace pose_graph {

class PoseGraphController {
  PoseGraphController(std::unique_ptr<Solver> optimizer)
      : solver_(std::move(optimizer)) {}

  PoseGraphController(const PoseGraphController&) = delete;
  PoseGraphController& operator=(const PoseGraphController&) = delete;

  void AddNode(const proto::Node& node) LOCKS_EXCLUDED(mutex_);
  void AddConstraint(const proto::Constraint& constraint)
      LOCKS_EXCLUDED(mutex_);

  Solver::SolverStatus Optimize() LOCKS_EXCLUDED(mutex_);

 private:
  std::unique_ptr<Solver> solver_;

  mutable absl::Mutex mutex_;
  PoseGraphData data_ GUARDED_BY(mutex_);
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_POSE_GRAPH_CONTROLLER_H_
