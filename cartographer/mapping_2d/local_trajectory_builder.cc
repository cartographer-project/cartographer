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

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/laser.h"

namespace cartographer {
namespace mapping_2d {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_horizontal_laser_min_z(
      parameter_dictionary->GetDouble("horizontal_laser_min_z"));
  options.set_horizontal_laser_max_z(
      parameter_dictionary->GetDouble("horizontal_laser_max_z"));
  options.set_horizontal_laser_voxel_filter_size(
      parameter_dictionary->GetDouble("horizontal_laser_voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() =
      mapping_3d::CreateMotionFilterOptions(
          parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_tracker_options() =
      kalman_filter::CreatePoseTrackerOptions(
          parameter_dictionary->GetDictionary("pose_tracker").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(options.submaps_options()),
      scan_matcher_pose_estimate_(transform::Rigid3d::Identity()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

const Submaps* LocalTrajectoryBuilder::submaps() const { return &submaps_; }

sensor::LaserFan LocalTrajectoryBuilder::BuildCroppedLaserFan(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan) const {
  const sensor::LaserFan cropped_fan = sensor::CropLaserFan(
      sensor::TransformLaserFan(laser_fan, tracking_to_tracking_2d),
      options_.horizontal_laser_min_z(), options_.horizontal_laser_max_z());
  return sensor::LaserFan{
      cropped_fan.origin,
      sensor::VoxelFiltered(cropped_fan.returns,
                            options_.horizontal_laser_voxel_filter_size()),
      sensor::VoxelFiltered(cropped_fan.misses,
                            options_.horizontal_laser_voxel_filter_size())};
}

void LocalTrajectoryBuilder::ScanMatch(
    common::Time time, const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan_in_tracking_2d,
    transform::Rigid3d* pose_observation,
    kalman_filter::PoseCovariance* covariance_observation) {
  const ProbabilityGrid& probability_grid =
      submaps_.Get(submaps_.matching_index())->probability_grid;
  transform::Rigid2d pose_prediction_2d =
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction_2d;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  const sensor::PointCloud filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(laser_fan_in_tracking_2d.returns);
  if (options_.use_online_correlative_scan_matching()) {
    real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        probability_grid, &initial_ceres_pose);
  }

  transform::Rigid2d tracking_2d_to_map;
  kalman_filter::Pose2DCovariance covariance_observation_2d;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      transform::Project2D(scan_matcher_pose_estimate_ *
                           tracking_to_tracking_2d.inverse()),
      initial_ceres_pose, filtered_point_cloud_in_tracking_2d, probability_grid,
      &tracking_2d_to_map, &covariance_observation_2d, &summary);

  CHECK(pose_tracker_ != nullptr);

  *pose_observation = transform::Embed3D(tracking_2d_to_map);
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);
  pose_tracker_->AddPoseObservation(
      time, (*pose_observation) * tracking_to_tracking_2d,
      *covariance_observation);
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan& laser_fan) {
  // Initialize pose tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializePoseTracker(time);
  }

  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // compute the orientation of the laser scanner.
    LOG(INFO) << "PoseTracker not yet initialized.";
    return nullptr;
  }

  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_prediction,
                                                  &covariance_prediction);

  // Computes the rotation without yaw, as defined by GetYaw().
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());

  const sensor::LaserFan laser_fan_in_tracking_2d =
      BuildCroppedLaserFan(tracking_to_tracking_2d.cast<float>(), laser_fan);

  if (laser_fan_in_tracking_2d.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal laser point cloud.";
    return nullptr;
  }

  transform::Rigid3d pose_observation;
  kalman_filter::PoseCovariance covariance_observation;
  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_observation,
            &covariance_observation);

  kalman_filter::PoseCovariance covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &scan_matcher_pose_estimate_, &covariance_estimate);

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = scan_matcher_pose_estimate_.translation();
  scan_matcher_pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      scan_matcher_pose_estimate_.rotation());

  const transform::Rigid3d tracking_2d_to_map =
      scan_matcher_pose_estimate_ * tracking_to_tracking_2d.inverse();
  last_pose_estimate_ = {
      time,
      {pose_prediction, covariance_prediction},
      {pose_observation, covariance_observation},
      {scan_matcher_pose_estimate_, covariance_estimate},
      scan_matcher_pose_estimate_,
      sensor::TransformPointCloud(laser_fan_in_tracking_2d.returns,
                                  tracking_2d_to_map.cast<float>())};

  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d))) {
    return nullptr;
  }

  const mapping::Submap* const matching_submap =
      submaps_.Get(submaps_.matching_index());
  std::vector<const mapping::Submap*> insertion_submaps;
  for (int insertion_index : submaps_.insertion_indices()) {
    insertion_submaps.push_back(submaps_.Get(insertion_index));
  }
  submaps_.InsertLaserFan(TransformLaserFan(laser_fan_in_tracking_2d,
                                            tracking_2d_to_map.cast<float>()));

  return common::make_unique<InsertionResult>(InsertionResult{
      time, &submaps_, matching_submap, insertion_submaps,
      tracking_to_tracking_2d, tracking_2d_to_map, laser_fan_in_tracking_2d,
      pose_estimate_2d, covariance_estimate});
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializePoseTracker(time);
  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  transform::Rigid3d pose_estimate;
  kalman_filter::PoseCovariance unused_covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate,
                                                  &unused_covariance_estimate);

  // Log a warning if the backpack inclination exceeds 20 degrees. In these
  // cases, it's very likely that 2D SLAM will fail.
  const Eigen::Vector3d gravity_direction =
      Eigen::Quaterniond(pose_estimate.rotation()) * Eigen::Vector3d::UnitZ();
  const double inclination = std::acos(gravity_direction.z());
  constexpr double kMaxInclination = common::DegToRad(20.);
  LOG_IF_EVERY_N(WARNING, inclination > kMaxInclination, 1000)
      << "Max inclination exceeded: " << common::RadToDeg(inclination) << " > "
      << common::RadToDeg(kMaxInclination);
}

void LocalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // process odometry poses.
    LOG_EVERY_N(INFO, 100) << "PoseTracker not yet initialized.";
  } else {
    pose_tracker_->AddOdometerPoseObservation(time, pose, covariance);
  }
}

void LocalTrajectoryBuilder::InitializePoseTracker(const common::Time time) {
  if (pose_tracker_ == nullptr) {
    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options_.pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k2D, time);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
