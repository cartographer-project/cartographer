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

#ifndef CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class GlobalTrajectoryBuilderInterface {
 public:
  // Represents a newly computed pose. Each of the following steps in the pose
  // estimation pipeline are provided for debugging:
  //
  // 1. UKF prediction
  // 2. Absolute pose observation (e.g. from scan matching)
  // 3. UKF estimate after integrating any measurements
  //
  // Finally, 'pose' is the end-user visualization of orientation and
  // 'point_cloud' is the point cloud, in the local map frame.
  struct PoseEstimate {
    PoseEstimate() = default;
    PoseEstimate(common::Time time,
                 const kalman_filter::PoseAndCovariance& prediction,
                 const kalman_filter::PoseAndCovariance& observation,
                 const kalman_filter::PoseAndCovariance& estimate,
                 const transform::Rigid3d& pose,
                 const sensor::PointCloud& point_cloud);

    common::Time time = common::Time::min();
    kalman_filter::PoseAndCovariance prediction = {
        transform::Rigid3d::Identity(), kalman_filter::PoseCovariance::Zero()};
    kalman_filter::PoseAndCovariance observation = {
        transform::Rigid3d::Identity(), kalman_filter::PoseCovariance::Zero()};
    kalman_filter::PoseAndCovariance estimate = {
        transform::Rigid3d::Identity(), kalman_filter::PoseCovariance::Zero()};
    transform::Rigid3d pose = transform::Rigid3d::Identity();
    sensor::PointCloud point_cloud;
  };

  GlobalTrajectoryBuilderInterface() {}
  virtual ~GlobalTrajectoryBuilderInterface() {}

  GlobalTrajectoryBuilderInterface(const GlobalTrajectoryBuilderInterface&) =
      delete;
  GlobalTrajectoryBuilderInterface& operator=(
      const GlobalTrajectoryBuilderInterface&) = delete;

  virtual const Submaps* submaps() const = 0;
  virtual Submaps* submaps() = 0;
  virtual kalman_filter::PoseTracker* pose_tracker() const = 0;
  virtual const PoseEstimate& pose_estimate() const = 0;

  virtual void AddLaserFan(common::Time time,
                           const sensor::LaserFan& laser_fan) = 0;
  virtual void AddImuData(common::Time time,
                          const Eigen::Vector3d& linear_acceleration,
                          const Eigen::Vector3d& angular_velocity) = 0;
  virtual void AddOdometerPose(
      common::Time time, const transform::Rigid3d& pose,
      const kalman_filter::PoseCovariance& covariance) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_GLOBAL_TRAJECTORY_BUILDER_INTERFACE_H_
