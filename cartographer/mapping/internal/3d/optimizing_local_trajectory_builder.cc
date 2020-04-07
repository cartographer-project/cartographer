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
#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/mapping/internal/3d/imu_static_calibration.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_direct_imu_integration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_imu_preintegration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/relative_translation_and_yaw_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {
class GravityDirectionCostFunction {
 public:
  GravityDirectionCostFunction(
      const double scaling_factor,
      const Eigen::Quaterniond& gravity_direction_in_tracking)
      : scaling_factor_(scaling_factor),
        gravity_direction_in_tracking_(gravity_direction_in_tracking) {}

  GravityDirectionCostFunction(const GravityDirectionCostFunction&) = delete;
  GravityDirectionCostFunction& operator=(const GravityDirectionCostFunction&) =
      delete;

  template <typename T>
  bool operator()(const T* const orientation, T* residual) const {
    Eigen::Quaternion<T> qorientation(orientation[0], orientation[1],
                                      orientation[2], orientation[3]);

    Eigen::Quaternion<T> delta =
        gravity_direction_in_tracking_.cast<T>() * qorientation;
    residual[0] = scaling_factor_ * transform::GetRoll(delta);
    residual[1] = scaling_factor_ * transform::GetPitch(delta);
    return true;
  }

 private:
  const double scaling_factor_;
  const Eigen::Quaterniond gravity_direction_in_tracking_;
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
      last_optimization_time_(common::FromUniversal(0)),
      initial_data_time_(common::FromUniversal(0)),
      optimization_rate_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .optimization_rate())),
      ct_window_horizon_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_horizon())),
      ct_window_rate_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_rate())),
      imu_calibrated_(false),
      linear_acceleration_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      angular_velocity_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      motion_filter_(options.motion_filter_options()),
      map_update_enabled_(true) {
  imu_integrator_ = absl::make_unique<ImuIntegrator>(
      options.optimizing_local_trajectory_builder_options().imu_integrator());
}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    imu_data_.push_back(imu_data);
    return;
  }
  initial_data_time_ = imu_data.time;
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
  imu_data_.push_back(imu_data);
}

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return;
  }
  if (!imu_data_.empty() && (imu_data_.front().time >= odometry_data.time)) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Odometry data dropped to maintain IMU consistency.";
    return;
  }
  odometer_data_.push_back(odometry_data);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  CHECK_GT(range_data_in_tracking.ranges.size(), 0);

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }
  PointCloudSet point_cloud_set;
  point_cloud_set.time = range_data_in_tracking.time;
  point_cloud_set.origin = range_data_in_tracking.origin;
  for (const auto& hit : range_data_in_tracking.ranges) {
    const Eigen::Vector3f delta = hit.position - range_data_in_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        point_cloud_set.points.push_back({hit.position});
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
  point_cloud_set.high_resolution_filtered_points =
      high_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);

  auto low_resolution_options =
      options_.low_resolution_adaptive_voxel_filter_options();
  low_resolution_options.set_min_num_points(
      low_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      low_resolution_options);
  point_cloud_set.low_resolution_filtered_points =
      low_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);
  point_cloud_data_.push_back(point_cloud_set);

  ++num_accumulated_;
  ++total_num_accumulated_;

  return MaybeOptimize(range_data_in_tracking.time);
}

void OptimizingLocalTrajectoryBuilder::AddControlPoint(common::Time t) {
  if (control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
        .initialize_map_orientation_with_imu()) {
      Eigen::Quaterniond g = extrapolator_->EstimateGravityOrientation(t);
      LOG(INFO) << "g " << g.vec();
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), g, Eigen::Vector3d::Zero())});
    } else {
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero())});
    }
  } else {
    if (active_submaps_.submaps().empty()) {
      control_points_.push_back(
          ControlPoint{t, control_points_.back().state});
    } else {
      control_points_.push_back(
          ControlPoint{t, PredictState(control_points_.back().state,
                                       control_points_.back().time, t)});
    }
  }
}

void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (control_points_.empty()) {
    return;
  }

  while (ct_window_horizon_ <
      control_points_.back().time - control_points_.front().time) {
    control_points_.pop_front();
  }

  while (point_cloud_data_.size() > 0 &&
      (point_cloud_data_.front().time < control_points_.front().time)) {
    point_cloud_data_.pop_front();
  }
  while (imu_data_.size() > 1 &&
      (imu_data_[1].time <= control_points_.front().time)) {
    imu_data_.pop_front();
  }

  while (odometer_data_.size() > 1 &&
         (odometer_data_[1].time <= control_points_.front().time)) {
    odometer_data_.pop_front();
  }
}

void OptimizingLocalTrajectoryBuilder::TransformStates(
    const transform::Rigid3d& transform) {
  for (ControlPoint& control_point : control_points_) {
    const transform::Rigid3d new_pose =
        transform * control_point.state.ToRigid();
    const auto& velocity = control_point.state.velocity;
    const Eigen::Vector3d new_velocity =
        transform.rotation() *
        Eigen::Vector3d(velocity[0], velocity[1], velocity[2]);
    control_point.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  if (time - initial_data_time_ < 2.0 * ct_window_horizon_ ||
      time - last_optimization_time_ < optimization_rate_) {
    if (time - initial_data_time_ < 2.0 * ct_window_horizon_) {
      LOG(INFO) << "No Optimization - not enough time since initialization "
                << common::ToSeconds(time - initial_data_time_) << "\t < "
                << 2.0 * common::ToSeconds(ct_window_horizon_);
    }
    return nullptr;
  }
  if(control_points_.empty()) {
    AddControlPoint(initial_data_time_);
  }
  last_optimization_time_ = time;
  if (!imu_calibrated_ &&
      options_.optimizing_local_trajectory_builder_options()
          .calibrate_imu()) {
    CalibrateIMU(imu_data_, gravity_constant_,
                 linear_acceleration_calibration_,
                 angular_velocity_calibration_);
    imu_integrator_->SetLinearAccelerationCalibration(
        linear_acceleration_calibration_);
    imu_integrator_->SetAngularVelocityCalibration(
        angular_velocity_calibration_);
    imu_calibrated_ = true;

    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    sensor::ImuData debiased_imu_data = imu_data_.front();
    debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
    debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
    extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
        ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
        options_.imu_gravity_time_constant(), debiased_imu_data);
    for(size_t i = 1; i < imu_data_.size(); ++i) {
      debiased_imu_data = imu_data_[i];
      debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
      debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
      extrapolator_->AddImuData(debiased_imu_data);
    }

  }
  if(options_.optimizing_local_trajectory_builder_options().sync_control_points_with_range_data()) {
    for (size_t i = 0; i < point_cloud_data_.size(); ++i) {
      PointCloudSet &point_cloud_set = point_cloud_data_[i];
      if(control_points_.back().time < point_cloud_set.time &&point_cloud_set.time < imu_data_.back().time) AddControlPoint(point_cloud_set.time);
    }
  }
  else {
    while (control_points_.back().time + ct_window_rate_ <
        imu_data_.back().time) {
      AddControlPoint(control_points_.back().time + ct_window_rate_);
    }
  }

  ceres::Problem problem;
  if (!active_submaps_.submaps().empty()
  ) {
    std::shared_ptr<const Submap3D> matching_submap =
        active_submaps_.submaps().front();


    // We assume the map is always aligned with the direction of gravity
    CHECK(matching_submap->local_pose().inverse().rotation().isApprox(Eigen::Quaterniond::Identity(), 1e-8));
    Eigen::Vector3d gravity =
        gravity_constant_ * Eigen::Vector3d::UnitZ();
    // We transform the states in 'control_points_' in place to be in the submap
    // frame as expected by the OccupiedSpaceCostFunctor. This is reverted after
    // solving the optimization problem.
    TransformStates(matching_submap->local_pose().inverse());
    auto next_control_point = control_points_.begin();
    for (size_t i = 0; i < point_cloud_data_.size(); ++i) {
      PointCloudSet& point_cloud_set = point_cloud_data_[i];
      if (point_cloud_set.time < control_points_.back().time) {
        while (next_control_point->time <= point_cloud_set.time) {
          CHECK(next_control_point != control_points_.end());
          next_control_point++;
        }
        CHECK(next_control_point != control_points_.begin());
        CHECK_LE(std::prev(next_control_point)->time, point_cloud_set.time);
        CHECK_GE(next_control_point->time, point_cloud_set.time);
        const double duration = common::ToSeconds(
            next_control_point->time - std::prev(next_control_point)->time);
        const double interpolation_factor =
            common::ToSeconds(point_cloud_set.time -
                              std::prev(next_control_point)->time) /
            duration;
        switch (matching_submap->high_resolution_hybrid_grid().GetGridType()) {
          case GridType::PROBABILITY_GRID: {
            problem.AddResidualBlock(
                scan_matching::InterpolatedOccupiedSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.high_resolution_filtered_points
                                    .size())),
                        point_cloud_set.high_resolution_filtered_points,
                        static_cast<const HybridGrid&>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
            break;
          }
          case GridType::TSDF: {
            if (options_.optimizing_local_trajectory_builder_options()
                    .high_resolution_grid_weight() > 0.0) {
              if(std::prev(next_control_point)->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D::
                  CreateAutoDiffCostFunction(
                      options_.optimizing_local_trajectory_builder_options()
                          .high_resolution_grid_weight() /
                          std::sqrt(static_cast<double>(
                                        point_cloud_set
                                            .high_resolution_filtered_points.size())),
                      point_cloud_set.high_resolution_filtered_points,
                      static_cast<const HybridGridTSDF&>(
                          matching_submap->high_resolution_hybrid_grid())),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data());
              }
              else {
                problem.AddResidualBlock(
                    scan_matching::InterpolatedTSDFSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                            .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                          point_cloud_set
                                              .high_resolution_filtered_points.size())),
                        point_cloud_set.high_resolution_filtered_points,
                        static_cast<const HybridGridTSDF &>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                    nullptr,
                    std::prev(next_control_point)->state.translation.data(),
                    std::prev(next_control_point)->state.rotation.data(),
                    next_control_point->state.translation.data(),
                    next_control_point->state.rotation.data());
              }
            }
            if (options_.optimizing_local_trajectory_builder_options()
                    .low_resolution_grid_weight() > 0.0) {

              if(std::prev(next_control_point)->time == point_cloud_set.time) {
                problem.AddResidualBlock(
                    scan_matching::TSDFSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                            .low_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                          point_cloud_set
                                              .low_resolution_filtered_points.size())),
                        point_cloud_set.low_resolution_filtered_points,
                        static_cast<const HybridGridTSDF&>(
                            matching_submap->low_resolution_hybrid_grid())),
                    nullptr,
                    std::prev(next_control_point)->state.translation.data(),
                    std::prev(next_control_point)->state.rotation.data());
              }
              else {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedTSDFSpaceCostFunction3D::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .low_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set.low_resolution_filtered_points
                                      .size())),
                          point_cloud_set.low_resolution_filtered_points,
                          static_cast<const HybridGridTSDF&>(
                              matching_submap->low_resolution_hybrid_grid()),
                          interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            }
            }
            break;
          }
          case GridType::NONE:
            LOG(FATAL) << "Gridtype not initialized.";
            break;
        }
      }
    }

    switch (options_.optimizing_local_trajectory_builder_options()
                .imu_cost_term()) {
      case proto::DIRECT: {
        for (size_t i = 1; i < control_points_.size(); ++i) {
          problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<
                  PredictionDirectImuIntegrationCostFunctor, 9, 3, 3, 4, 3, 3,
                  4>(new PredictionDirectImuIntegrationCostFunctor(
                  options_.optimizing_local_trajectory_builder_options()
                      .translation_weight(),
                  options_.optimizing_local_trajectory_builder_options()
                      .velocity_weight(),
                  options_.optimizing_local_trajectory_builder_options()
                      .rotation_weight(),
                  common::ToSeconds(control_points_[i].time -
                                    control_points_[i - 1].time),
                  imu_data_, linear_acceleration_calibration_,
                  angular_velocity_calibration_, control_points_[i - 1].time,
                  control_points_[i].time,
                  options_.optimizing_local_trajectory_builder_options()
                      .imu_integrator())),
              nullptr, control_points_[i - 1].state.translation.data(),
              control_points_[i - 1].state.velocity.data(),
              control_points_[i - 1].state.rotation.data(),
              control_points_[i].state.translation.data(),
              control_points_[i].state.velocity.data(),
              control_points_[i].state.rotation.data());
        }
        break;
      }
      case proto::PREINTEGRATION: {
        auto it = --imu_data_.cend();
        while (it->time > control_points_.begin()->time) {
          CHECK(it != imu_data_.cbegin());
          --it;
        }
        for (size_t i = 1; i < control_points_.size(); ++i) {
          IntegrateImuWithTranslationResult<double> result =
              imu_integrator_->IntegrateIMU(imu_data_,
                                            control_points_[i - 1].time,
                                            control_points_[i].time, &it);

          problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<
                  PredictionImuPreintegrationCostFunctor, 9, 3, 3, 4, 3, 3, 4>(
                  new PredictionImuPreintegrationCostFunctor(
                      options_.optimizing_local_trajectory_builder_options()
                          .translation_weight(),
                      options_.optimizing_local_trajectory_builder_options()
                          .velocity_weight(),
                      options_.optimizing_local_trajectory_builder_options()
                          .rotation_weight(),
                      common::ToSeconds(control_points_[i].time -
                                        control_points_[i - 1].time),
                      result,
                      gravity)),
              nullptr, control_points_[i - 1].state.translation.data(),
              control_points_[i - 1].state.velocity.data(),
              control_points_[i - 1].state.rotation.data(),
              control_points_[i].state.translation.data(),
              control_points_[i].state.velocity.data(),
              control_points_[i].state.rotation.data());
        }
        break;
      }
      default:
        LOG(FATAL) << "Unknown proto::IMUCostTerm";
    }

    if (odometer_data_.size() > 1) {
      transform::TransformInterpolationBuffer interpolation_buffer;
      for (const auto& odometer_data : odometer_data_) {
        interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
      }
      for (size_t i = 1; i < control_points_.size(); ++i) {
        // Only add constraints for this range data if  we have bracketing data
        // from the odometer.
        if (!(interpolation_buffer.earliest_time() <=
                  control_points_[i - 1].time &&
              control_points_[i].time <= interpolation_buffer.latest_time())) {
          continue;
        }
        const transform::Rigid3d previous_odometer_pose =
            interpolation_buffer.Lookup(control_points_[i - 1].time);
        const transform::Rigid3d current_odometer_pose =
            interpolation_buffer.Lookup(control_points_[i].time);
        const transform::Rigid3d delta_pose =
            current_odometer_pose.inverse() * previous_odometer_pose;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                RelativeTranslationAndYawCostFunction, 6, 3, 4, 3, 4>(
                new RelativeTranslationAndYawCostFunction(
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_translation_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_rotation_weight(),
                    delta_pose)),
            nullptr, control_points_[i - 1].state.translation.data(),
            control_points_[i - 1].state.rotation.data(),
            control_points_[i].state.translation.data(),
            control_points_[i].state.rotation.data());
      }
    }

    problem.SetParameterBlockConstant(
        control_points_.front().state.translation.data());
    problem.SetParameterBlockConstant(
        control_points_.front().state.rotation.data());
    problem.SetParameterBlockConstant(
        control_points_.front().state.velocity.data());

    for (size_t i = 1; i < control_points_.size(); ++i) {
      problem.SetParameterization(control_points_[i].state.rotation.data(),
                                  new ceres::QuaternionParameterization());

      problem.SetParameterBlockConstant(
          control_points_[i].state.rotation.data());
    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_solver_options_, &problem, &summary);
    // The optimized states in 'control_points_' are in the submap frame and we
    // transform them in place to be in the local SLAM frame again.
    TransformStates(matching_submap->local_pose());
  }


  num_accumulated_ = 0;
  const transform::Rigid3d optimized_pose =
      control_points_.front().state.ToRigid();
  extrapolator_->AddPose(control_points_.front().time, optimized_pose);
  sensor::RangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}};

  if (active_submaps_.submaps().empty()) {
    //To initialize the empty map we add all available range data assuming zero motion.
    std::deque<ControlPoint>::iterator control_points_iterator =
        control_points_.begin();
    for (auto& point_cloud_set : point_cloud_data_) {
      if (point_cloud_set.time < control_points_.back().time) {
        while (control_points_iterator->time <= point_cloud_set.time) {
          ++control_points_iterator;
        }
        CHECK(control_points_iterator != control_points_.begin());
        CHECK(control_points_iterator != control_points_.end());
        auto transform_cloud = InterpolateTransform(
            std::prev(control_points_iterator)->state.ToRigid(),
            control_points_iterator->state.ToRigid(),
            std::prev(control_points_iterator)->time,
            control_points_iterator->time, point_cloud_set.time);
        const transform::Rigid3f transform =
            (optimized_pose.inverse() * transform_cloud).cast<float>();
        for (const auto& point : point_cloud_set.points) {
          accumulated_range_data_in_tracking.returns.push_back(transform *
                                                               point);
        }
        accumulated_range_data_in_tracking.origin =
            (transform * point_cloud_set.origin);
      }
    }
  } else {
    CHECK(control_points_.front().time <= point_cloud_data_.front().time);
    while (ct_window_horizon_ <
           control_points_.back().time - point_cloud_data_.front().time) {
      while (std::next(control_points_.begin())->time <
             point_cloud_data_.front().time) {
        control_points_.pop_front();
      }
      CHECK(std::next(control_points_.begin()) != control_points_.end());
      auto transform_cloud = InterpolateTransform(
          control_points_.begin()->state.ToRigid(),
          std::next(control_points_.begin())->state.ToRigid(),
          control_points_.begin()->time,
          std::next(control_points_.begin())->time,
          point_cloud_data_.front().time);
      const transform::Rigid3f transform =
          (optimized_pose.inverse() * transform_cloud).cast<float>();
      for (const auto& point : point_cloud_data_.front().points) {
        accumulated_range_data_in_tracking.returns.push_back(transform * point);
      }
      accumulated_range_data_in_tracking.origin =
          (transform * point_cloud_data_.front().origin);
      point_cloud_data_.pop_front();
    }
  }

  RemoveObsoleteSensorData();

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

  const Eigen::Quaterniond gravity_alignment = optimized_pose.rotation();
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
  if (!map_update_enabled_) {
    LOG(WARNING) << "Map Update Disabled!";
  }
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      map_update_enabled_
          ? active_submaps_.InsertData(
                filtered_range_data_in_local, local_from_gravity_aligned,
                rotational_scan_matcher_histogram_in_gravity)
          : active_submaps_.submaps();
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

State OptimizingLocalTrajectoryBuilder::PredictState(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  switch (
      options_.optimizing_local_trajectory_builder_options().imu_integrator()) {
    case proto::IMUIntegrator::EULER:
      return PredictStateEuler(start_state, start_time, end_time);
    case proto::IMUIntegrator::RK4:
      return PredictStateRK4(start_state, start_time, end_time);
    default:
      LOG(FATAL) << "Unsupported imu integrator type.";
  }
}

State OptimizingLocalTrajectoryBuilder::PredictStateRK4(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }
  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateStateWithGravity(imu_data_, start_state,
                                                 start_time, end_time, &it);
  return State(result.delta_translation, result.delta_rotation,
               result.delta_velocity);
}

State OptimizingLocalTrajectoryBuilder::PredictStateEuler(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }

  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateIMU(imu_data_, start_time, end_time, &it);

  const Eigen::Quaterniond start_rotation(
      start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
      start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * result.delta_rotation;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

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

void OptimizingLocalTrajectoryBuilder::SetMapUpdateEnabled(
    bool map_update_enabled) {
  map_update_enabled_ = map_update_enabled;
}

}  // namespace mapping
}  // namespace cartographer
