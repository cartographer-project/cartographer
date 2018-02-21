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

#include "cartographer/metrics/register.h"

#include "cartographer/mapping_2d/pose_graph/constraint_builder.h"
#include "cartographer/mapping_3d/pose_graph/constraint_builder.h"

namespace cartographer {
namespace metrics {

void RegisterAllMetrics(FamilyFactory* registry) {
  mapping_2d::pose_graph::ConstraintBuilder::RegisterMetrics(registry);
  mapping_3d::pose_graph::ConstraintBuilder::RegisterMetrics(registry);
}

}  // namespace metrics
}  // namespace cartographer
