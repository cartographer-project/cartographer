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

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include <string>

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

// This type is a logical union, i.e. only one type of sensor data is actually
// filled in. It is only used for time ordering sensor data before passing it
// on.
struct Data {
  enum class Type { kImu, kLaserFan, kOdometry };

  struct Odometry {
    transform::Rigid3d pose;
    kalman_filter::PoseCovariance covariance;
  };

  struct Imu {
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };

  Data(const string& frame_id, const Imu& imu)
      : type(Type::kImu), frame_id(frame_id), imu(imu) {}

  Data(const string& frame_id,
       const ::cartographer::sensor::LaserFan& laser_fan)
      : type(Type::kLaserFan), frame_id(frame_id), laser_fan(laser_fan) {}

  Data(const string& frame_id, const Odometry& odometry)
      : type(Type::kOdometry), frame_id(frame_id), odometry(odometry) {}

  Type type;
  string frame_id;
  Imu imu;
  sensor::LaserFan laser_fan;
  Odometry odometry;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
