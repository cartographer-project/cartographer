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

#ifndef CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
#define CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_

#include <deque>
#include <memory>

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "cartographer/kalman_filter/proto/pose_tracker_options.pb.h"
#include "cartographer/kalman_filter/unscented_kalman_filter.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/odometry_state_tracker.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace kalman_filter {

typedef Eigen::Matrix3d Pose2DCovariance;
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;

struct PoseAndCovariance {
  transform::Rigid3d pose;
  PoseCovariance covariance;
};

PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance);

// Projects a 3D pose covariance into a 2D pose covariance.
Pose2DCovariance Project2D(const PoseCovariance& embedded_covariance);

// Embeds a 2D pose covariance into a 3D pose covariance.
PoseCovariance Embed3D(const Pose2DCovariance& embedded_covariance,
                       double position_variance, double orientation_variance);

PoseCovariance BuildPoseCovariance(double translational_variance,
                                   double rotational_variance);

// Deserializes the 'proto_matrix' into a PoseCovariance.
PoseCovariance PoseCovarianceFromProtoMatrix(
    const sensor::proto::Matrix& proto_matrix);

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A Kalman filter for a 3D state of position and orientation.
// Includes functions to update from IMU and range data matches.
class PoseTracker {
 public:
  enum {
    kMapPositionX = 0,
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  // We terminate loops with this.
  };

  enum class ModelFunction { k2D, k3D };

  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;
  using State = KalmanFilter::StateType;
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;
  using Distribution = GaussianDistribution<double, kDimension>;

  // Create a new Kalman filter starting at the origin with a standard normal
  // distribution at 'time'.
  PoseTracker(const proto::PoseTrackerOptions& options,
              const ModelFunction& model_function, common::Time time);
  virtual ~PoseTracker();

  // Sets 'pose' and 'covariance' to the mean and covariance of the belief at
  // 'time' which must be >= to the current time. Must not be nullptr.
  void GetPoseEstimateMeanAndCovariance(common::Time time,
                                        transform::Rigid3d* pose,
                                        PoseCovariance* covariance);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      common::Time time, const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      common::Time time, const Eigen::Vector3d& imu_angular_velocity);

  // Updates from a pose estimate in the map frame.
  void AddPoseObservation(common::Time time, const transform::Rigid3d& pose,
                          const PoseCovariance& covariance);

  // Updates from a pose estimate in the odometer's map-like frame.
  void AddOdometerPoseObservation(common::Time time,
                                  const transform::Rigid3d& pose,
                                  const PoseCovariance& covariance);

  common::Time time() const { return time_; }

  // Returns the belief at the 'time' which must be >= to the current time.
  Distribution GetBelief(common::Time time);

  const mapping::OdometryStateTracker::OdometryStates& odometry_states() const;

 private:
  // Returns the distribution required to initialize the KalmanFilter.
  static Distribution KalmanFilterInit();

  // Build a model noise distribution (zero mean) for the given time delta.
  const Distribution BuildModelNoise(double delta_t) const;

  // Predict the state forward in time. This is a no-op if 'time' has not moved
  // forward.
  void Predict(common::Time time);

  // Computes a pose combining the given 'state' with the 'imu_tracker_'
  // orientation.
  transform::Rigid3d RigidFromState(const PoseTracker::State& state);

  const proto::PoseTrackerOptions options_;
  const ModelFunction model_function_;
  common::Time time_;
  KalmanFilter kalman_filter_;
  mapping::ImuTracker imu_tracker_;
  mapping::OdometryStateTracker odometry_state_tracker_;
};

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
