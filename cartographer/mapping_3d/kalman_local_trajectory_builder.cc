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

#include "cartographer/mapping_3d/kalman_local_trajectory_builder.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/proto/pose_tracker_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/proto/real_time_correlative_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/proto/submaps_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

KalmanLocalTrajectoryBuilder::KalmanLocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(common::make_unique<Submaps>(options.submaps_options())),
      scan_matcher_pose_estimate_(transform::Rigid3d::Identity()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher>(
              options_.kalman_local_trajectory_builder_options()
                  .real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(common::make_unique<scan_matching::CeresScanMatcher>(
          options_.ceres_scan_matcher_options())),
      num_accumulated_(0),
      first_pose_prediction_(transform::Rigid3f::Identity()),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}} {}

KalmanLocalTrajectoryBuilder::~KalmanLocalTrajectoryBuilder() {}

const mapping_3d::Submaps* KalmanLocalTrajectoryBuilder::submaps() const {
  return submaps_.get();
}

void KalmanLocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  if (!pose_tracker_) {
    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options_.kalman_local_trajectory_builder_options()
            .pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k3D, time);
  }

  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  transform::Rigid3d pose_estimate;
  kalman_filter::PoseCovariance unused_covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate,
                                                  &unused_covariance_estimate);
}

std::unique_ptr<KalmanLocalTrajectoryBuilder::InsertionResult>
KalmanLocalTrajectoryBuilder::AddRangefinderData(
    const common::Time time, const Eigen::Vector3f& origin,
    const sensor::PointCloud& ranges, int next_trajectory_node_index) {
  if (!pose_tracker_) {
    LOG(INFO) << "PoseTracker not yet initialized.";
    return nullptr;
  }

  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance unused_covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &pose_prediction, &unused_covariance_prediction);
  if (num_accumulated_ == 0) {
    first_pose_prediction_ = pose_prediction.cast<float>();
    accumulated_range_data_ =
        sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}};
  }

  const transform::Rigid3f tracking_delta =
      first_pose_prediction_.inverse() * pose_prediction.cast<float>();
  const sensor::RangeData range_data_in_first_tracking =
      sensor::TransformRangeData(sensor::RangeData{origin, ranges, {}, {}},
                                 tracking_delta);
  for (const Eigen::Vector3f& hit : range_data_in_first_tracking.returns) {
    const Eigen::Vector3f delta = hit - range_data_in_first_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.laser_min_range()) {
      if (range <= options_.laser_max_range()) {
        accumulated_range_data_.returns.push_back(hit);
      } else {
        // We insert a ray cropped to 'laser_max_range' as a miss for hits
        // beyond the maximum range. This way the free space up to the maximum
        // range will be updated.
        accumulated_range_data_.misses.push_back(
            range_data_in_first_tracking.origin +
            options_.laser_max_range() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.scans_per_accumulation()) {
    num_accumulated_ = 0;
    return AddAccumulatedRangeData(
        time,
        sensor::TransformRangeData(accumulated_range_data_,
                                   tracking_delta.inverse()),
        next_trajectory_node_index);
  }
  return nullptr;
}

std::unique_ptr<KalmanLocalTrajectoryBuilder::InsertionResult>
KalmanLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const sensor::RangeData& range_data_in_tracking,
    const int trajectory_node_index) {
  const sensor::RangeData filtered_range_data = {
      range_data_in_tracking.origin,
      sensor::VoxelFiltered(range_data_in_tracking.returns,
                            options_.laser_voxel_filter_size()),
      sensor::VoxelFiltered(range_data_in_tracking.misses,
                            options_.laser_voxel_filter_size())};

  if (filtered_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance unused_covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &pose_prediction, &unused_covariance_prediction);

  transform::Rigid3d initial_ceres_pose = pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data.returns);
  if (options_.kalman_local_trajectory_builder_options()
          .use_online_correlative_scan_matching()) {
    real_time_correlative_scan_matcher_->Match(
        pose_prediction, filtered_point_cloud_in_tracking,
        submaps_->high_resolution_matching_grid(), &initial_ceres_pose);
  }

  transform::Rigid3d pose_observation;
  ceres::Solver::Summary summary;

  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(filtered_range_data.returns);
  ceres_scan_matcher_->Match(scan_matcher_pose_estimate_, initial_ceres_pose,
                             {{&filtered_point_cloud_in_tracking,
                               &submaps_->high_resolution_matching_grid()},
                              {&low_resolution_point_cloud_in_tracking,
                               &submaps_->low_resolution_matching_grid()}},
                             &pose_observation, &summary);
  pose_tracker_->AddPoseObservation(
      time, pose_observation,
      options_.kalman_local_trajectory_builder_options()
              .scan_matcher_variance() *
          kalman_filter::PoseCovariance::Identity());

  kalman_filter::PoseCovariance covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &scan_matcher_pose_estimate_, &covariance_estimate);

  last_pose_estimate_ = {
      time, scan_matcher_pose_estimate_,
      sensor::TransformPointCloud(filtered_range_data.returns,
                                  pose_observation.cast<float>())};

  return InsertIntoSubmap(time, filtered_range_data, pose_observation,
                          covariance_estimate, trajectory_node_index);
}

void KalmanLocalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& pose) {
  if (!pose_tracker_) {
    pose_tracker_.reset(new kalman_filter::PoseTracker(
        options_.kalman_local_trajectory_builder_options()
            .pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k3D, time));
  }
  pose_tracker_->AddOdometerPoseObservation(
      time, pose,
      kalman_filter::BuildPoseCovariance(
          options_.kalman_local_trajectory_builder_options()
              .odometer_translational_variance(),
          options_.kalman_local_trajectory_builder_options()
              .odometer_rotational_variance()));
}

const KalmanLocalTrajectoryBuilder::PoseEstimate&
KalmanLocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

std::unique_ptr<KalmanLocalTrajectoryBuilder::InsertionResult>
KalmanLocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_tracking,
    const transform::Rigid3d& pose_observation,
    const kalman_filter::PoseCovariance& covariance_estimate,
    const int trajectory_node_index) {
  if (motion_filter_.IsSimilar(time, pose_observation)) {
    return nullptr;
  }
  const Submap* const matching_submap =
      submaps_->Get(submaps_->matching_index());
  std::vector<const Submap*> insertion_submaps;
  for (int insertion_index : submaps_->insertion_indices()) {
    insertion_submaps.push_back(submaps_->Get(insertion_index));
  }
  submaps_->InsertRangeData(
      sensor::TransformRangeData(range_data_in_tracking,
                                 pose_observation.cast<float>()),
      trajectory_node_index);
  return std::unique_ptr<InsertionResult>(new InsertionResult{
      time, range_data_in_tracking, pose_observation, covariance_estimate,
      submaps_.get(), matching_submap, insertion_submaps});
}

}  // namespace mapping_3d
}  // namespace cartographer
