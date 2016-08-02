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

#include "cartographer/mapping/global_trajectory_builder_interface.h"

namespace cartographer {
namespace mapping {

GlobalTrajectoryBuilderInterface::PoseEstimate::PoseEstimate(
    const common::Time time, const kalman_filter::PoseAndCovariance& prediction,
    const kalman_filter::PoseAndCovariance& observation,
    const kalman_filter::PoseAndCovariance& estimate,
    const transform::Rigid3d& pose, const sensor::PointCloud& point_cloud)
    : time(time),
      prediction(prediction),
      observation(observation),
      estimate(estimate),
      pose(pose),
      point_cloud(point_cloud) {}

}  // namespace mapping
}  // namespace cartographer
