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
  options.set_laser_min_range(
      parameter_dictionary->GetDouble("laser_min_range"));
  options.set_laser_max_range(
      parameter_dictionary->GetDouble("laser_max_range"));
  options.set_laser_min_z(parameter_dictionary->GetDouble("laser_min_z"));
  options.set_laser_max_z(parameter_dictionary->GetDouble("laser_max_z"));
  options.set_laser_missing_echo_ray_length(
      parameter_dictionary->GetDouble("laser_missing_echo_ray_length"));
  options.set_laser_voxel_filter_size(
      parameter_dictionary->GetDouble("laser_voxel_filter_size"));
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
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  options.set_num_odometry_states(
      parameter_dictionary->GetNonNegativeInt("num_odometry_states"));
  CHECK_GT(options.num_odometry_states(), 0);
  *options.mutable_submaps_options() = CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      odometry_state_tracker_(options_.num_odometry_states()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

const Submaps* LocalTrajectoryBuilder::submaps() const { return &submaps_; }

sensor::LaserFan LocalTrajectoryBuilder::TransformAndFilterLaserFan(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan) const {
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  sensor::LaserFan returns_and_misses{laser_fan.origin, {}, {}, {}};
  for (const Eigen::Vector3f& return_ : laser_fan.returns) {
    const float range = (return_ - laser_fan.origin).norm();
    if (range >= options_.laser_min_range()) {
      if (range <= options_.laser_max_range()) {
        returns_and_misses.returns.push_back(return_);
      } else {
        returns_and_misses.misses.push_back(
            laser_fan.origin + options_.laser_missing_echo_ray_length() *
                                   (return_ - laser_fan.origin).normalized());
      }
    }
  }
  const sensor::LaserFan cropped = sensor::CropLaserFan(
      sensor::TransformLaserFan(returns_and_misses, tracking_to_tracking_2d),
      options_.laser_min_z(), options_.laser_max_z());
  return sensor::LaserFan{
      cropped.origin,
      sensor::VoxelFiltered(cropped.returns,
                            options_.laser_voxel_filter_size()),
      sensor::VoxelFiltered(cropped.misses,
                            options_.laser_voxel_filter_size())};
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
  ceres_scan_matcher_.Match(pose_prediction_2d, initial_ceres_pose,
                            filtered_point_cloud_in_tracking_2d,
                            probability_grid, &tracking_2d_to_map,
                            &covariance_observation_2d, &summary);

  *pose_observation =
      transform::Embed3D(tracking_2d_to_map) * tracking_to_tracking_2d;
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan& laser_fan) {
  // Initialize IMU tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeImuTracker(time);
  }

  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker with our first IMU message, we
    // cannot compute the orientation of the laser scanner.
    LOG(INFO) << "ImuTracker not yet initialized.";
    return nullptr;
  }

  Predict(time);
  const transform::Rigid3d odometry_prediction =
      pose_estimate_ * odometry_correction_;
  const transform::Rigid3d model_prediction = pose_estimate_;
  // TODO(whess): Prefer IMU over odom orientation if configured?
  const transform::Rigid3d& pose_prediction = odometry_prediction;

  // Computes the rotation without yaw, as defined by GetYaw().
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());

  const sensor::LaserFan laser_fan_in_tracking_2d = TransformAndFilterLaserFan(
      tracking_to_tracking_2d.cast<float>(), laser_fan);

  if (laser_fan_in_tracking_2d.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal laser point cloud.";
    return nullptr;
  }

  kalman_filter::PoseCovariance covariance_observation;
  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_estimate_, &covariance_observation);
  odometry_correction_ = transform::Rigid3d::Identity();
  if (!odometry_state_tracker_.empty()) {
    // We add an odometry state, so that the correction from the scan matching
    // is not removed by the next odometry data we get.
    odometry_state_tracker_.AddOdometryState(
        {time, odometry_state_tracker_.newest().odometer_pose,
         odometry_state_tracker_.newest().state_pose *
             odometry_prediction.inverse() * pose_estimate_});
  }

  // Improve the velocity estimate.
  if (last_scan_match_time_ > common::Time::min()) {
    const double delta_t = common::ToSeconds(time - last_scan_match_time_);
    velocity_estimate_ += (pose_estimate_.translation().head<2>() -
                           model_prediction.translation().head<2>()) /
                          delta_t;
  }
  last_scan_match_time_ = time_;

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = pose_estimate_.translation();
  pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      pose_estimate_.rotation());

  const transform::Rigid3d tracking_2d_to_map =
      pose_estimate_ * tracking_to_tracking_2d.inverse();
  last_pose_estimate_ = {
      time, pose_estimate_,
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
  submaps_.InsertLaserFan(
      TransformLaserFan(laser_fan_in_tracking_2d,
                        transform::Embed3D(pose_estimate_2d.cast<float>())));

  return common::make_unique<InsertionResult>(InsertionResult{
      time, &submaps_, matching_submap, insertion_submaps,
      tracking_to_tracking_2d, tracking_2d_to_map, laser_fan_in_tracking_2d,
      pose_estimate_2d, covariance_observation});
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializeImuTracker(time);
  Predict(time);
  imu_tracker_->AddImuLinearAccelerationObservation(linear_acceleration);
  imu_tracker_->AddImuAngularVelocityObservation(angular_velocity);

  // Log a warning if the backpack inclination exceeds 20 degrees. In these
  // cases, it's very likely that 2D SLAM will fail.
  const Eigen::Vector3d gravity_direction =
      imu_tracker_->orientation() * Eigen::Vector3d::UnitZ();
  const double inclination = std::acos(gravity_direction.z());
  constexpr double kMaxInclination = common::DegToRad(20.);
  LOG_IF_EVERY_N(WARNING, inclination > kMaxInclination, 1000)
      << "Max inclination exceeded: " << common::RadToDeg(inclination) << " > "
      << common::RadToDeg(kMaxInclination);
}

void LocalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& odometer_pose) {
  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker we do not want to call Predict().
    LOG(INFO) << "ImuTracker not yet initialized.";
    return;
  }

  Predict(time);
  if (!odometry_state_tracker_.empty()) {
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose;
    const transform::Rigid3d new_pose =
        previous_odometry_state.state_pose * delta;
    odometry_correction_ = pose_estimate_.inverse() * new_pose;
  }
  odometry_state_tracker_.AddOdometryState(
      {time, odometer_pose, pose_estimate_ * odometry_correction_});
}

void LocalTrajectoryBuilder::InitializeImuTracker(const common::Time time) {
  if (imu_tracker_ == nullptr) {
    imu_tracker_ = common::make_unique<mapping::ImuTracker>(
        options_.imu_gravity_time_constant(), time);
  }
}

void LocalTrajectoryBuilder::Predict(const common::Time time) {
  CHECK(imu_tracker_ != nullptr);
  CHECK_LE(time_, time);
  const double last_yaw = transform::GetYaw(imu_tracker_->orientation());
  imu_tracker_->Advance(time);
  if (time_ > common::Time::min()) {
    const double delta_t = common::ToSeconds(time - time_);
    // Constant velocity model.
    const Eigen::Vector3d translation =
        pose_estimate_.translation() +
        delta_t *
            Eigen::Vector3d(velocity_estimate_.x(), velocity_estimate_.y(), 0.);
    // Use the current IMU tracker roll and pitch for gravity alignment, and
    // apply its change in yaw.
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(
            transform::GetYaw(pose_estimate_.rotation()) - last_yaw,
            Eigen::Vector3d::UnitZ()) *
        imu_tracker_->orientation();
    pose_estimate_ = transform::Rigid3d(translation, rotation);
  }
  time_ = time;
}

}  // namespace mapping_2d
}  // namespace cartographer
