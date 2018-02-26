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

#include "cartographer/internal/mapping/2d/local_trajectory_builder_2d.h"
#include "cartographer/internal/mapping/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/2d/pose_graph_2d.h"
#include "cartographer/mapping/3d/pose_graph_3d.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_MAPPING_GLOBAL_TRAJECTORY_BUILDER_H_
