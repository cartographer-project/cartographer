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

#include "cartographer/pose_graph/optimizer/ceres_optimizer.h"

namespace cartographer {
namespace pose_graph {
namespace {

ceres::Problem::Options CreateCeresProblemOptions() {
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  return problem_options;
}

Optimizer::SolverStatus ToSolverStatus(
    const ceres::TerminationType& termination_type) {
  switch (termination_type) {
    case (ceres::TerminationType::CONVERGENCE):
      return Optimizer::SolverStatus::CONVERGENCE;
    case (ceres::TerminationType::NO_CONVERGENCE):
      return Optimizer::SolverStatus::NO_CONVERGENCE;
    default:
      return Optimizer::SolverStatus::FAILURE;
  }
}

}  // namespace

CeresOptimizer::CeresOptimizer(const ceres::Solver::Options& options)
    : problem_options_(CreateCeresProblemOptions()), solver_options_(options) {}

Optimizer::SolverStatus CeresOptimizer::Solve(PoseGraphData* data) const {
  ceres::Problem problem(problem_options_);

  for (const auto& constraint : data->constraints) {
    constraint->AddToOptimizer(&data->nodes, &problem);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options_, &problem, &summary);
  return ToSolverStatus(summary.termination_type);
}

}  // namespace pose_graph
}  // namespace cartographer
