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

#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <>
void Dispatchable<sensor::ImuData>::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilder* const trajectory_builder,
    const std::string& sensor_id) {
  trajectory_builder->AddImuData(
      sensor_id, data_.time, data_.linear_acceleration, data_.angular_velocity);
}

template <>
void Dispatchable<sensor::OdometryData>::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilder* const trajectory_builder,
    const std::string& sensor_id) {
  trajectory_builder->AddOdometerData(sensor_id, data_.time, data_.pose);
}

template <>
void Dispatchable<sensor::FixedFramePoseData>::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilder* const trajectory_builder,
    const std::string& sensor_id) {
  trajectory_builder->AddFixedFramePoseData(sensor_id, data_.time, data_.pose);
}

}  // namespace sensor
}  // namespace cartographer
