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

#ifndef CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_

#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_2d/local_trajectory_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"

namespace cartographer {
namespace mapping_2d {

class GlobalTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options,
                          int trajectory_id,
                          SparsePoseGraph* sparse_pose_graph);
  ~GlobalTrajectoryBuilder() override;

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  const Submaps* submaps() const override;
  const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate& pose_estimate()
      const override;

  // Projects 'ranges' into 2D. Therefore, 'ranges' should be approximately
  // parallel to the ground plane.
  void AddRangefinderData(common::Time time, const Eigen::Vector3f& origin,
                          const sensor::PointCloud& ranges) override;
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;
  void AddOdometerData(common::Time time,
                       const transform::Rigid3d& pose) override;

 private:
  const int trajectory_id_;
  SparsePoseGraph* const sparse_pose_graph_;
  LocalTrajectoryBuilder local_trajectory_builder_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_
