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
#include "cartographer/mapping/pose_estimate.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
class LocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  void AddImuData(const sensor::ImuData& imu_data);
  std::unique_ptr<InsertionResult> AddRangeData(
      common::Time time, const sensor::RangeData& range_data);
  void AddOdometerData(const sensor::OdometryData& odometry_data);
  const mapping::PoseEstimate& pose_estimate() const;

 private:
  std::unique_ptr<InsertionResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_tracking,
      const Eigen::Quaterniond& gravity_alignment,
      const sensor::PointCloud& high_resolution_point_cloud,
      const sensor::PointCloud& low_resolution_point_cloud,
      const Eigen::VectorXf& rotational_scan_matcher_histogram,
      const transform::Rigid3d& pose_observation);

  const proto::LocalTrajectoryBuilderOptions options_;
  ActiveSubmaps active_submaps_;

  mapping::PoseEstimate last_pose_estimate_;

  MotionFilter motion_filter_;
  std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher>
      real_time_correlative_scan_matcher_;
  std::unique_ptr<scan_matching::CeresScanMatcher> ceres_scan_matcher_;

  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  transform::Rigid3f first_pose_estimate_ = transform::Rigid3f::Identity();
  sensor::RangeData accumulated_range_data_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
