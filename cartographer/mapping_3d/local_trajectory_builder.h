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

#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/odometry_state_tracker.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {

// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
// closure.
class LocalTrajectoryBuilder {
 public:
  using PoseEstimate = mapping::GlobalTrajectoryBuilderInterface::PoseEstimate;

  struct InsertionResult {
    common::Time time;
    sensor::RangeData range_data_in_tracking;
    transform::Rigid3d pose_observation;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  std::unique_ptr<InsertionResult> AddRangefinderData(
      common::Time time, const Eigen::Vector3f& origin,
      const sensor::PointCloud& ranges);
  void AddOdometerData(common::Time time,
                       const transform::Rigid3d& odometer_pose);
  const PoseEstimate& pose_estimate() const;

 private:
  void Predict(common::Time time);

  std::unique_ptr<InsertionResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_tracking,
      const transform::Rigid3d& pose_observation);

  const proto::LocalTrajectoryBuilderOptions options_;
  ActiveSubmaps active_submaps_;

  PoseEstimate last_pose_estimate_;

  MotionFilter motion_filter_;
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher>
      real_time_correlative_scan_matcher_;
  std::unique_ptr<scan_matching::CeresScanMatcher> ceres_scan_matcher_;

  // Current 'pose_estimate_' and 'velocity_estimate_' at 'time_'.
  common::Time time_ = common::Time::min();
  transform::Rigid3d pose_estimate_ = transform::Rigid3d::Identity();
  Eigen::Vector3d velocity_estimate_ = Eigen::Vector3d::Zero();
  common::Time last_scan_match_time_ = common::Time::min();
  // This is the difference between the model (constant velocity, IMU)
  // prediction 'pose_estimate_' and the odometry prediction. To get the
  // odometry prediction, right-multiply this to 'pose_estimate_'.
  transform::Rigid3d odometry_correction_ = transform::Rigid3d::Identity();
  std::unique_ptr<mapping::ImuTracker> imu_tracker_;
  mapping::OdometryStateTracker odometry_state_tracker_;

  int num_accumulated_ = 0;
  transform::Rigid3f first_pose_estimate_ = transform::Rigid3f::Identity();
  sensor::RangeData accumulated_range_data_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
