/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_

#include <memory>
#include <tuple>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/pose_extrapolator_options.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace mapping {

proto::PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class PoseExtrapolatorInterface {
 public:
  struct ExtrapolationResult {
    // The poses for the requested times at index 0 to N-1.
    std::vector<transform::Rigid3f> previous_poses;
    // The pose for the requested time at index N.
    transform::Rigid3d current_pose;
    Eigen::Vector3d current_velocity;
    Eigen::Quaterniond gravity_from_tracking;
  };

  PoseExtrapolatorInterface(const PoseExtrapolatorInterface&) = delete;
  PoseExtrapolatorInterface& operator=(const PoseExtrapolatorInterface&) =
      delete;
  virtual ~PoseExtrapolatorInterface() {}

  // TODO: Remove dependency cycle.
  static std::unique_ptr<PoseExtrapolatorInterface> CreateWithImuData(
      const proto::PoseExtrapolatorOptions& options,
      const std::vector<sensor::ImuData>& imu_data,
      const std::vector<transform::TimestampedTransform>& initial_poses);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  virtual common::Time GetLastPoseTime() const = 0;
  virtual common::Time GetLastExtrapolatedTime() const = 0;

  virtual void AddPose(common::Time time, const transform::Rigid3d& pose) = 0;
  virtual void AddImuData(const sensor::ImuData& imu_data) = 0;
  virtual void AddOdometryData(const sensor::OdometryData& odometry_data) = 0;
  virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;

  virtual ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) = 0;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  virtual Eigen::Quaterniond EstimateGravityOrientation(common::Time time) = 0;

 protected:
  PoseExtrapolatorInterface() {}
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_
