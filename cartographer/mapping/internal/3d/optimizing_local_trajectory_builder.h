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

#ifndef CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_

#include <array>
#include <chrono>
#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/pose_extrapolator.h"

#include "cartographer/metrics/family_factory.h"

#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct State {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // Rotation quaternion as (w, x, y, z).
  std::array<double, 3> velocity;

  State(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& velocity)
      : translation{{translation.x(), translation.y(), translation.z()}},
        rotation{{rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
        velocity{{velocity.x(), velocity.y(), velocity.z()}} {}

  Eigen::Quaterniond ToQuaternion() const {
    return Eigen::Quaterniond(rotation[0], rotation[1], rotation[2],
                              rotation[3]);
  }

  transform::Rigid3d ToRigid() const {
    return transform::Rigid3d(
        Eigen::Vector3d(translation[0], translation[1], translation[2]),
        ToQuaternion());
  }
};
// Batches up some sensor data and optimizes them in one go to get a locally
// consistent trajectory.
class OptimizingLocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  };

  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit OptimizingLocalTrajectoryBuilder(
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~OptimizingLocalTrajectoryBuilder();

  OptimizingLocalTrajectoryBuilder(const OptimizingLocalTrajectoryBuilder&) =
      delete;
  OptimizingLocalTrajectoryBuilder& operator=(
      const OptimizingLocalTrajectoryBuilder&) = delete;

  void AddImuData(const sensor::ImuData& imu_data);

  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data_in_tracking);

  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

  void SetMapUpdateEnabled(bool map_update_enabled);

 private:
  void AddControlPoint(common::Time t);

  struct PointCloudSet {
    common::Time time;
    Eigen::Vector3f origin;
    sensor::PointCloud points;
    sensor::PointCloud high_resolution_filtered_points;
    sensor::PointCloud low_resolution_filtered_points;
  };

  struct ControlPoint {
    common::Time time;
    State state;
  };

  State PredictState(const State& start_state, const common::Time start_time,
                     const common::Time end_time);

  State PredictStateRK4(const State& start_state, const common::Time start_time,
                        const common::Time end_time);

  void RemoveObsoleteSensorData();

  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const transform::Rigid3d& pose_observation,
      const sensor::RangeData& range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& filtered_range_data_in_local,
      const sensor::RangeData& filtered_range_data_in_tracking,
      const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
      const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  void TransformStates(const transform::Rigid3d& transform);
  std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
  MaybeOptimize(common::Time time);

  const proto::LocalTrajectoryBuilderOptions3D options_;
  const ceres::Solver::Options ceres_solver_options_;
  mapping::ActiveSubmaps3D active_submaps_;
  int num_accumulated_;
  int total_num_accumulated_;
  common::Time last_optimization_time_;
  common::Time initial_data_time_;
  common::Duration optimization_rate_;

  std::deque<ControlPoint> control_points_;
  double gravity_constant_ = 9.80665;
  std::deque<sensor::ImuData> imu_data_;
  std::deque<sensor::OdometryData> odometer_data_;
  std::deque<PointCloudSet> point_cloud_data_;

  common::Duration ct_window_horizon_;
  common::Duration ct_window_rate_;

  bool imu_calibrated_;
  Eigen::Transform<double, 3, Eigen::Affine> linear_acceleration_calibration_;
  Eigen::Transform<double, 3, Eigen::Affine> angular_velocity_calibration_;

  MotionFilter motion_filter_;
  std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;
  bool map_update_enabled_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_
