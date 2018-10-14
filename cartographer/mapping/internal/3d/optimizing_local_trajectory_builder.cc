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

#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder.h"

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

// Computes the cost of differences between two velocities. For the constant
// velocity model the residuals are just the vector difference.
class VelocityDeltaCostFunctor {
 public:
  explicit VelocityDeltaCostFunctor(const double scaling_factor)
      : scaling_factor_(scaling_factor) {}

  VelocityDeltaCostFunctor(const VelocityDeltaCostFunctor&) = delete;
  VelocityDeltaCostFunctor& operator=(const VelocityDeltaCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const old_velocity, const T* const new_velocity,
                  T* residual) const {
    residual[0] = scaling_factor_ * (new_velocity[0] - old_velocity[0]);
    residual[1] = scaling_factor_ * (new_velocity[1] - old_velocity[1]);
    residual[2] = scaling_factor_ * (new_velocity[2] - old_velocity[2]);
    return true;
  }

 private:
  const double scaling_factor_;
};

class RelativeTranslationAndYawCostFunction {
 public:
  RelativeTranslationAndYawCostFunction(const double translation_scaling_factor,
                                        const double rotation_scaling_factor,
                                        const transform::Rigid3d& delta)
      : translation_scaling_factor_(translation_scaling_factor),
        rotation_scaling_factor_(rotation_scaling_factor),
        delta_(delta) {}

  RelativeTranslationAndYawCostFunction(
      const RelativeTranslationAndYawCostFunction&) = delete;
  RelativeTranslationAndYawCostFunction& operator=(
      const RelativeTranslationAndYawCostFunction&) = delete;

  template <typename T>
  bool operator()(const T* const start_translation,
                  const T* const start_rotation, const T* const end_translation,
                  const T* const end_rotation, T* residual) const {
    const transform::Rigid3<T> start(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_translation),
        Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                             start_rotation[2], start_rotation[3]));
    const transform::Rigid3<T> end(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_translation),
        Eigen::Quaternion<T>(end_rotation[0], end_rotation[1], end_rotation[2],
                             end_rotation[3]));

    const transform::Rigid3<T> delta = end.inverse() * start;
    const transform::Rigid3<T> error = delta.inverse() * delta_.cast<T>();
    residual[0] = translation_scaling_factor_ * error.translation().x();
    residual[1] = translation_scaling_factor_ * error.translation().y();
    residual[2] = translation_scaling_factor_ * error.translation().z();
    residual[3] = rotation_scaling_factor_ * transform::GetYaw(error);
    return true;
  }

 private:
  const double translation_scaling_factor_;
  const double rotation_scaling_factor_;
  const transform::Rigid3d delta_;
};

}  // namespace

OptimizingLocalTrajectoryBuilder::OptimizingLocalTrajectoryBuilder(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      ceres_solver_options_(common::CreateCeresSolverOptions(
          options.ceres_scan_matcher_options().ceres_solver_options())),
      active_submaps_(options.submaps_options()),
      num_accumulated_(0),
      total_num_accumulated_(0),
      motion_filter_(options.motion_filter_options()) {}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    imu_data_.push_back(imu_data);
    RemoveObsoleteSensorData();
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

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  odometer_data_.push_back(odometry_data);
  RemoveObsoleteSensorData();
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data) {
  CHECK_GT(range_data.ranges.size(), 0);

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }
  const common::Time time_first_point =
      range_data.time + common::FromSeconds(range_data.ranges.front().time);
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    // return nullptr;
  }

  // TODO(hrapp): Handle misses.
  // TODO(hrapp): Where are NaNs in range_data_in_tracking coming from?
  sensor::PointCloud point_cloud;
  for (const auto& hit : range_data.ranges) {
    const Eigen::Vector3f delta = hit.position - range_data.origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        point_cloud.push_back({hit.position});
      }
    }
  }

  auto high_resolution_options =
      options_.high_resolution_adaptive_voxel_filter_options();
  high_resolution_options.set_min_num_points(
      high_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter high_resolution_adaptive_voxel_filter(
      high_resolution_options);
  const sensor::PointCloud high_resolution_filtered_points =
      high_resolution_adaptive_voxel_filter.Filter(point_cloud);

  auto low_resolution_options =
      options_.low_resolution_adaptive_voxel_filter_options();
  low_resolution_options.set_min_num_points(
      low_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      low_resolution_options);
  const sensor::PointCloud low_resolution_filtered_points =
      low_resolution_adaptive_voxel_filter.Filter(point_cloud);

  if (batches_.empty()) {
    // First rangefinder data ever. Initialize to the origin.
    batches_.push_back(
        Batch{range_data.time, point_cloud, high_resolution_filtered_points,
              low_resolution_filtered_points,
              State(Eigen::Vector3d::Zero(),
                    extrapolator_->EstimateGravityOrientation(range_data.time),
                    Eigen::Vector3d::Zero())});
  } else {
    const Batch& last_batch = batches_.back();
    batches_.push_back(Batch{
        range_data.time,
        point_cloud,
        high_resolution_filtered_points,
        low_resolution_filtered_points,
        PredictState(last_batch.state, last_batch.time, range_data.time),
    });
  }
  ++num_accumulated_;
  ++total_num_accumulated_;

  RemoveObsoleteSensorData();
  return MaybeOptimize(range_data.time);
}

void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (imu_data_.empty()) {
    batches_.clear();
    return;
  }

  while (batches_.size() >
         static_cast<size_t>(options_.num_accumulated_range_data())) {
    batches_.pop_front();
  }

  while (imu_data_.size() > 1 &&
         (batches_.empty() || imu_data_[1].time <= batches_.front().time)) {
    imu_data_.pop_front();
  }

  while (
      odometer_data_.size() > 1 &&
      (batches_.empty() || odometer_data_[1].time <= batches_.front().time)) {
    odometer_data_.pop_front();
  }
}

void OptimizingLocalTrajectoryBuilder::TransformStates(
    const transform::Rigid3d& transform) {
  for (Batch& batch : batches_) {
    const transform::Rigid3d new_pose = transform * batch.state.ToRigid();
    const auto& velocity = batch.state.velocity;
    const Eigen::Vector3d new_velocity =
        transform.rotation() *
        Eigen::Vector3d(velocity[0], velocity[1], velocity[2]);
    batch.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  if ((num_accumulated_ < options_.num_accumulated_range_data() &&
       num_accumulated_ % options_.optimizing_local_trajectory_builder_options()
                              .scans_per_optimization_update() !=
           0) ||
      total_num_accumulated_ < options_.num_accumulated_range_data()) {
    return nullptr;
  }

  ceres::Problem problem;
  if (!active_submaps_.submaps().empty() &&
      (total_num_accumulated_ >= options_.num_accumulated_range_data())) {
    std::shared_ptr<const Submap3D> matching_submap =
        active_submaps_.submaps().front();

    // We transform the states in 'batches_' in place to be in the submap frame
    // as expected by the OccupiedSpaceCostFunctor. This is reverted after
    // solving the optimization problem.
    TransformStates(matching_submap->local_pose().inverse());
    for (size_t i = 0; i < batches_.size(); ++i) {
      Batch& batch = batches_[i];
      problem.AddResidualBlock(
          scan_matching::OccupiedSpaceCostFunction3D::
              CreateAutoDiffCostFunction(
                  options_.optimizing_local_trajectory_builder_options()
                          .high_resolution_grid_weight() /
                      std::sqrt(static_cast<double>(
                          batch.high_resolution_filtered_points.size())),
                  batch.high_resolution_filtered_points,
                  matching_submap->high_resolution_hybrid_grid()),
          nullptr, batch.state.translation.data(), batch.state.rotation.data());
      problem.AddResidualBlock(
          scan_matching::OccupiedSpaceCostFunction3D::
              CreateAutoDiffCostFunction(
                  options_.optimizing_local_trajectory_builder_options()
                          .low_resolution_grid_weight() /
                      std::sqrt(static_cast<double>(
                          batch.low_resolution_filtered_points.size())),
                  batch.low_resolution_filtered_points,
                  matching_submap->low_resolution_hybrid_grid()),
          nullptr, batch.state.translation.data(), batch.state.rotation.data());
      if (i == 0) {
        problem.SetParameterBlockConstant(batch.state.translation.data());
        problem.SetParameterBlockConstant(batch.state.rotation.data());
        problem.AddParameterBlock(batch.state.velocity.data(), 3);
        problem.SetParameterBlockConstant(batch.state.velocity.data());
      } else {
        problem.SetParameterization(batch.state.rotation.data(),
                                    new ceres::QuaternionParameterization());
      }
    }

    auto it = imu_data_.cbegin();
    for (size_t i = 1; i < batches_.size(); ++i) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<VelocityDeltaCostFunctor, 3, 3, 3>(
              new VelocityDeltaCostFunctor(
                  options_.optimizing_local_trajectory_builder_options()
                      .velocity_weight())),
          nullptr, batches_[i - 1].state.velocity.data(),
          batches_[i].state.velocity.data());

      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<TranslationCostFunction, 3, 3, 3, 3>(
              new TranslationCostFunction(
                  options_.optimizing_local_trajectory_builder_options()
                      .translation_weight(),
                  common::ToSeconds(batches_[i].time - batches_[i - 1].time))),
          nullptr, batches_[i - 1].state.translation.data(),
          batches_[i].state.translation.data(),
          batches_[i - 1].state.velocity.data());

      const IntegrateImuResult<double> result =
          IntegrateImu(imu_data_, batches_[i - 1].time, batches_[i].time, &it);
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<RotationCostFunction, 3, 4, 4>(
              new RotationCostFunction(
                  options_.optimizing_local_trajectory_builder_options()
                      .rotation_weight(),
                  result.delta_rotation)),
          nullptr, batches_[i - 1].state.rotation.data(),
          batches_[i].state.rotation.data());
    }

    if (odometer_data_.size() > 1) {
      transform::TransformInterpolationBuffer interpolation_buffer;
      for (const auto& odometer_data : odometer_data_) {
        interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
      }
      for (size_t i = 1; i < batches_.size(); ++i) {
        // Only add constraints for this range data if  we have bracketing data
        // from the odometer.
        if (!(interpolation_buffer.earliest_time() <= batches_[i - 1].time &&
              batches_[i].time <= interpolation_buffer.latest_time())) {
          continue;
        }
        const transform::Rigid3d previous_odometer_pose =
            interpolation_buffer.Lookup(batches_[i - 1].time);
        const transform::Rigid3d current_odometer_pose =
            interpolation_buffer.Lookup(batches_[i].time);
        const transform::Rigid3d delta_pose =
            current_odometer_pose.inverse() * previous_odometer_pose;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                RelativeTranslationAndYawCostFunction, 4, 3, 4, 3, 4>(
                new RelativeTranslationAndYawCostFunction(
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_translation_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_rotation_weight(),
                    delta_pose)),
            nullptr, batches_[i - 1].state.translation.data(),
            batches_[i - 1].state.rotation.data(),
            batches_[i].state.translation.data(),
            batches_[i].state.rotation.data());
      }
    }

    ceres::Solver::Summary summary;
    ceres::Solve(ceres_solver_options_, &problem, &summary);
    // The optimized states in 'batches_' are in the submap frame and we
    // transform them in place to be in the local SLAM frame again.
    TransformStates(matching_submap->local_pose());
    if (num_accumulated_ %
                options_.optimizing_local_trajectory_builder_options()
                    .scans_per_optimization_update() !=
            0 &&
        active_submaps_.submaps().front()->num_range_data() <
            options_.num_accumulated_range_data()) {
      LOG(INFO) << "num_accumulated_ < options_.num_accumulated_range_data() "
                << num_accumulated_;
      return nullptr;
    }
  }

  num_accumulated_ = 0;

  const transform::Rigid3d optimized_pose = batches_.back().state.ToRigid();

  extrapolator_->AddPose(time, optimized_pose);
  sensor::RangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}};

  if (active_submaps_.submaps().empty()) {
    for (const auto& batch : batches_) {
      const transform::Rigid3f transform =
          (optimized_pose.inverse() * batch.state.ToRigid()).cast<float>();
      for (const auto& point : batch.points) {
        accumulated_range_data_in_tracking.returns.push_back(transform * point);
      }
    }
  } else {
    for (int i = options_.optimizing_local_trajectory_builder_options()
                     .scans_per_optimization_update() -
                 1;
         i >= 0; --i) {
      if (batches_.size() > i) {
        Batch batch = batches_[i];
        const transform::Rigid3f transform =
            (optimized_pose.inverse() * batch.state.ToRigid()).cast<float>();
        for (const auto& point : batch.points) {
          accumulated_range_data_in_tracking.returns.push_back(transform *
                                                               point);
        }
      }
    }
  }

  return AddAccumulatedRangeData(time, optimized_pose,
                                 accumulated_range_data_in_tracking);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const transform::Rigid3d& optimized_pose,
    const sensor::RangeData& range_data_in_tracking) {
  if (range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  sensor::RangeData filtered_range_data_in_tracking = {
      range_data_in_tracking.origin,
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.returns),
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.misses)};

  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, optimized_pose.cast<float>());

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  const Eigen::Quaterniond gravity_alignment =
      extrapolator_->EstimateGravityOrientation(time);
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, optimized_pose,
      gravity_alignment);

  return absl::make_unique<MatchingResult>(MatchingResult{
      time, optimized_pose, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::InsertIntoSubmap(
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
  const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());

  const Eigen::Quaterniond local_from_gravity_aligned =
      pose_estimate.rotation() * gravity_alignment.inverse();
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      active_submaps_.InsertData(filtered_range_data_in_local,
                                 local_from_gravity_aligned,
                                 rotational_scan_matcher_histogram_in_gravity);
  return absl::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram_in_gravity,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

OptimizingLocalTrajectoryBuilder::State
OptimizingLocalTrajectoryBuilder::PredictState(const State& start_state,
                                               const common::Time start_time,
                                               const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }

  const IntegrateImuResult<double> result =
      IntegrateImu(imu_data_, start_time, end_time, &it);

  const Eigen::Quaterniond start_rotation(
      start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
      start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * result.delta_rotation;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

  // TODO(hrapp): IntegrateImu should integration position as well.
  const Eigen::Vector3d position =
      Eigen::Map<const Eigen::Vector3d>(start_state.translation.data()) +
      delta_time_seconds *
          Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data());
  const Eigen::Vector3d velocity =
      Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data()) +
      start_rotation * result.delta_velocity -
      gravity_constant_ * delta_time_seconds * Eigen::Vector3d::UnitZ();

  return State(position, orientation, velocity);
}

void OptimizingLocalTrajectoryBuilder::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  LOG(WARNING)
      << "OptimizingLocalTrajectoryBuilder::RegisterMetrics not implemented";
}

}  // namespace mapping
}  // namespace cartographer