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

#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

// This type is a logical union, i.e. only one type of sensor data is actually
// filled in. It is only used for time ordering sensor data before passing it
// on.
struct Data {
  enum class Type { kImu, kRangefinder, kOdometer };

  struct Imu {
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };

  struct Rangefinder {
    Eigen::Vector3f origin;
    PointCloud ranges;
  };

  Data(const common::Time time, const Imu& imu)
      : type(Type::kImu), time(time), imu(imu) {}

  Data(const common::Time time, const Rangefinder& rangefinder)
      : type(Type::kRangefinder), time(time), rangefinder(rangefinder) {}

  Data(const common::Time time, const transform::Rigid3d& odometer_pose)
      : type(Type::kOdometer), time(time), odometer_pose(odometer_pose) {}

  Type type;
  common::Time time;
  Imu imu;
  Rangefinder rangefinder;
  transform::Rigid3d odometer_pose;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
