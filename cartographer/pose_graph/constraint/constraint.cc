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

#include "cartographer/pose_graph/constraint/constraint.h"

namespace cartographer {
namespace pose_graph {

proto::Constraint Constraint::ToProto() const {
  proto::Constraint constraint;
  constraint.set_id(constraint_id_);
  *constraint.mutable_cost_function() = ToCostFunctionProto();
  return constraint;
}

}  // namespace pose_graph
}  // namespace cartographer
