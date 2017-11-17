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

#include "cartographer/mapping_3d/local_trajectory_builder.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_2d/scan_matching/proto/real_time_correlative_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/proto/submaps_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/rotational_scan_matcher.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(common::make_unique<scan_matching::CeresScanMatcher>(
          options_.ceres_scan_matcher_options())),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}} {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

void LocalTrajectoryBuilder::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::AddRangeData(const common::Time time,
                                     const sensor::TimedRangeData& range_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }
  if (num_accumulated_ == 0) {
    first_pose_estimate_ = extrapolator_->ExtrapolatePose(time).cast<float>();
    accumulated_range_data_ = sensor::RangeData{range_data.origin, {}, {}};
  }

  // TODO(gaschler): Take time delta of individual points into account.
  const transform::Rigid3f tracking_delta =
      first_pose_estimate_.inverse() *
      extrapolator_->ExtrapolatePose(time).cast<float>();
  const Eigen::Vector3f origin_in_first_tracking =
      tracking_delta * range_data.origin;
  for (const Eigen::Vector4f& hit : range_data.returns) {
    const Eigen::Vector3f hit_in_first_tracking =
        tracking_delta * hit.head<3>();
    const Eigen::Vector3f delta =
        hit_in_first_tracking - origin_in_first_tracking;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_first_tracking);
      } else {
        // We insert a ray cropped to 'max_range' as a miss for hits beyond the
        // maximum range. This way the free space up to the maximum range will
        // be updated.
        accumulated_range_data_.misses.push_back(
            origin_in_first_tracking + options_.max_range() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    num_accumulated_ = 0;
    const sensor::RangeData filtered_range_data = {
        accumulated_range_data_.origin,
        sensor::VoxelFiltered(accumulated_range_data_.returns,
                              options_.voxel_filter_size()),
        sensor::VoxelFiltered(accumulated_range_data_.misses,
                              options_.voxel_filter_size())};
    return AddAccumulatedRangeData(
        time, sensor::TransformRangeData(filtered_range_data,
                                         tracking_delta.inverse()));
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking) {
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  const transform::Rigid3d pose_prediction =
      extrapolator_->ExtrapolatePose(time);

  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
  }

  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;

  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }
  ceres_scan_matcher_->Match(
      matching_submap->local_pose().inverse() * pose_prediction,
      initial_ceres_pose,
      {{&high_resolution_point_cloud_in_tracking,
        &matching_submap->high_resolution_hybrid_grid()},
       {&low_resolution_point_cloud_in_tracking,
        &matching_submap->low_resolution_hybrid_grid()}},
      &pose_observation_in_submap, &summary);
  const transform::Rigid3d pose_estimate =
      matching_submap->local_pose() * pose_observation_in_submap;
  extrapolator_->AddPose(time, pose_estimate);
  const Eigen::Quaterniond gravity_alignment =
      extrapolator_->EstimateGravityOrientation(time);

  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, pose_estimate.cast<float>());
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, pose_estimate, gravity_alignment);
  return common::make_unique<MatchingResult>(MatchingResult{
      time, pose_estimate, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

void LocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (const std::shared_ptr<Submap>& submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(filtered_range_data_in_local,
                                  gravity_alignment);
  const auto rotational_scan_matcher_histogram =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());
  return common::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

}  // namespace mapping_3d
}  // namespace cartographer
