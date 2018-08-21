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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/sensor/internal/trajectory_collator.h"

namespace cartographer {
namespace metrics {

void RegisterAllMetrics(FamilyFactory* registry) {
  mapping::constraints::ConstraintBuilder2D::RegisterMetrics(registry);
  mapping::constraints::ConstraintBuilder3D::RegisterMetrics(registry);
  mapping::GlobalTrajectoryBuilderRegisterMetrics(registry);
  mapping::LocalTrajectoryBuilder2D::RegisterMetrics(registry);
  mapping::LocalTrajectoryBuilder3D::RegisterMetrics(registry);
  mapping::PoseGraph2D::RegisterMetrics(registry);
  mapping::PoseGraph3D::RegisterMetrics(registry);
  sensor::TrajectoryCollator::RegisterMetrics(registry);
}

}  // namespace metrics
}  // namespace cartographer
