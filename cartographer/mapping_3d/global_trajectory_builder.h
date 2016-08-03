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

#ifndef CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/local_trajectory_builder.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"

namespace cartographer {
namespace mapping_3d {

class GlobalTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface {
 public:
  GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options,
                          mapping_3d::SparsePoseGraph* sparse_pose_graph);
  ~GlobalTrajectoryBuilder() override;

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  const mapping_3d::Submaps* submaps() const override;
  mapping_3d::Submaps* submaps() override;
  kalman_filter::PoseTracker* pose_tracker() const override;
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;
  void AddLaserFan3D(common::Time time,
                     const sensor::LaserFan3D& laser_fan) override;
  void AddOdometerPose(
      common::Time time, const transform::Rigid3d& pose,
      const kalman_filter::PoseCovariance& covariance) override;
  const PoseEstimate& pose_estimate() const override;

  void AddHorizontalLaserFan(common::Time, const sensor::LaserFan3D&) override {
    LOG(FATAL) << "Not implemented.";
  }

 private:
  mapping_3d::SparsePoseGraph* const sparse_pose_graph_;
  std::unique_ptr<LocalTrajectoryBuilderInterface> local_trajectory_builder_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_GLOBAL_TRAJECTORY_BUILDER_H_
