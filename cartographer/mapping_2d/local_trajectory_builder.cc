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

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>
#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping_2d {

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

sensor::RangeData LocalTrajectoryBuilder::TransformAndFilterRangeData(
    const transform::Rigid3f& gravity_alignment,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped = sensor::CropRangeData(
      sensor::TransformRangeData(range_data, gravity_alignment),
      options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFiltered(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFiltered(cropped.misses, options_.voxel_filter_size())};
}

void LocalTrajectoryBuilder::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::RangeData& gravity_aligned_range_data,
    transform::Rigid2d* const pose_observation) {
  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);
  if (options_.use_online_correlative_scan_matching()) {
    real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        matching_submap->probability_grid(), &initial_ceres_pose);
  }

  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      pose_prediction, initial_ceres_pose, filtered_gravity_aligned_point_cloud,
      matching_submap->probability_grid(), pose_observation, &summary);
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddRangeData(const common::Time time,
                                     const sensor::RangeData& range_data) {
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }
  if (num_accumulated_ == 0) {
    first_pose_estimate_ = extrapolator_->ExtrapolatePose(time).cast<float>();
    accumulated_range_data_ =
        sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}};
  }

  const transform::Rigid3f tracking_delta =
      first_pose_estimate_.inverse() *
      extrapolator_->ExtrapolatePose(time).cast<float>();
  const sensor::RangeData range_data_in_first_tracking =
      sensor::TransformRangeData(range_data, tracking_delta);
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (const Eigen::Vector3f& hit : range_data_in_first_tracking.returns) {
    const Eigen::Vector3f delta = hit - range_data_in_first_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit);
      } else {
        accumulated_range_data_.misses.push_back(
            range_data_in_first_tracking.origin +
            options_.missing_data_ray_length() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.scans_per_accumulation()) {
    num_accumulated_ = 0;
    return AddAccumulatedRangeData(
        time, sensor::TransformRangeData(accumulated_range_data_,
                                         tracking_delta.inverse()));
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const sensor::RangeData& range_data) {
  // Transforms 'range_data' into a frame where gravity direction is
  // approximately +z.
  const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
      extrapolator_->EstimateGravityOrientation(time));
  const sensor::RangeData gravity_aligned_range_data =
      TransformAndFilterRangeData(gravity_alignment.cast<float>(), range_data);
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  transform::Rigid2d pose_estimate_2d;
  ScanMatch(time, pose_prediction, gravity_aligned_range_data,
            &pose_estimate_2d);
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(pose_estimate_2d) * gravity_alignment;
  extrapolator_->AddPose(time, pose_estimate);

  last_pose_estimate_ = {
      time, pose_estimate,
      sensor::TransformPointCloud(
          gravity_aligned_range_data.returns,
          transform::Embed3D(pose_estimate_2d.cast<float>()))};

  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }

  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (const std::shared_ptr<Submap>& submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d.cast<float>())));

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.loop_closure_adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_gravity_aligned_point_cloud =
      adaptive_voxel_filter.Filter(gravity_aligned_range_data.returns);

  return common::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::TrajectoryNode::Data{
              time,
              {},  // 'range_data' is only used in 3D.
              filtered_gravity_aligned_point_cloud,
              {},  // 'high_resolution_point_cloud' is only used in 3D.
              {},  // 'low_resolution_point_cloud' is only used in 3D.
              gravity_alignment}),
      pose_estimate_2d, std::move(insertion_submaps)});
}

const mapping::PoseEstimate& LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

void LocalTrajectoryBuilder::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

void LocalTrajectoryBuilder::AddOdometerData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

void LocalTrajectoryBuilder::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = common::make_unique<mapping::PoseExtrapolator>(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant());
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

}  // namespace mapping_2d
}  // namespace cartographer
