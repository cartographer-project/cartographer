/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/common/ceres_solver_options.h"

namespace cartographer {
namespace common {

proto::CeresSolverOptions CreateCeresSolverOptionsProto(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::CeresSolverOptions proto;
  proto.set_use_nonmonotonic_steps(
      parameter_dictionary->GetBool("use_nonmonotonic_steps"));
  proto.set_max_num_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_iterations"));
  proto.set_num_threads(parameter_dictionary->GetNonNegativeInt("num_threads"));
  CHECK_GT(proto.max_num_iterations(), 0);
  CHECK_GT(proto.num_threads(), 0);
  return proto;
}

ceres::Solver::Options CreateCeresSolverOptions(
    const proto::CeresSolverOptions& proto) {
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = proto.use_nonmonotonic_steps();
  options.max_num_iterations = proto.max_num_iterations();
  options.num_threads = proto.num_threads();
  return options;
}

}  // namespace common
}  // namespace cartographer
