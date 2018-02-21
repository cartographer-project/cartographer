/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/internal/mapping_2d/local_trajectory_builder.h"
#include "cartographer/internal/mapping_3d/local_trajectory_builder.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping_2d/pose_graph_2d.h"
#include "cartographer/mapping_3d/pose_graph_3d.h"

namespace cartographer {
namespace mapping {

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<mapping_2d::LocalTrajectoryBuilder>
        local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<mapping_3d::LocalTrajectoryBuilder>
        local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
