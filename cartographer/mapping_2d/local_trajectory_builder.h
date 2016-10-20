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

#ifndef CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping_2d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/sensor/configuration.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
// closure.
class LocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    common::Time time;
    const mapping::Submaps* submaps;
    const mapping::Submap* matching_submap;
    std::vector<const mapping::Submap*> insertion_submaps;
    transform::Rigid3d tracking_to_tracking_2d;
    transform::Rigid3d tracking_2d_to_map;
    sensor::LaserFan laser_fan_in_tracking_2d;
    transform::Rigid2d pose_estimate_2d;
    kalman_filter::PoseCovariance covariance_estimate;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate& pose_estimate()
      const;
  std::unique_ptr<InsertionResult> AddHorizontalLaserFan(
      common::Time, const sensor::LaserFan& laser_fan);
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  void AddOdometerPose(common::Time time, const transform::Rigid3d& pose,
                       const kalman_filter::PoseCovariance& covariance);

  const Submaps* submaps() const;

 private:
  // Transforms 'laser_scan', crops and voxel filters.
  sensor::LaserFan BuildCroppedLaserFan(
      const transform::Rigid3f& tracking_to_tracking_2d,
      const sensor::LaserFan& laser_fan) const;

  // Scan match 'laser_fan_in_tracking_2d' and fill in the
  // 'pose_observation' and 'covariance_observation' with the result.
  void ScanMatch(common::Time time, const transform::Rigid3d& pose_prediction,
                 const transform::Rigid3d& tracking_to_tracking_2d,
                 const sensor::LaserFan& laser_fan_in_tracking_2d,
                 transform::Rigid3d* pose_observation,
                 kalman_filter::PoseCovariance* covariance_observation);

  // Lazily constructs a PoseTracker.
  void InitializePoseTracker(common::Time time);

  const proto::LocalTrajectoryBuilderOptions options_;
  Submaps submaps_;
  mapping::GlobalTrajectoryBuilderInterface::PoseEstimate last_pose_estimate_;

  // Pose of the last computed scan match.
  transform::Rigid3d scan_matcher_pose_estimate_;

  mapping_3d::MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  std::unique_ptr<kalman_filter::PoseTracker> pose_tracker_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
