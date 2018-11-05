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

#ifndef CARTOGRAPHER_POSE_GRAPH_SOLVER_SOLVER_H_
#define CARTOGRAPHER_POSE_GRAPH_SOLVER_SOLVER_H_

#include "cartographer/pose_graph/pose_graph_data.h"

namespace cartographer {
namespace pose_graph {

class Solver {
 public:
  enum class SolverStatus {
    CONVERGENCE,
    NO_CONVERGENCE,
    FAILURE,
  };

  Solver() = default;
  virtual ~Solver() = default;

  Solver(const Solver&) = delete;
  Solver& operator=(const Solver&) = delete;

  virtual SolverStatus Solve(PoseGraphData* data) const = 0;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_SOLVER_SOLVER_H_
