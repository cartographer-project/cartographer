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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_TEST_HELPERS_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_TEST_HELPERS_H_

#include <string>
#include <tuple>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/dispatchable.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {

MATCHER_P(Near, point, std::string(negation ? "Doesn't" : "Does") + " match.") {
  return arg.isApprox(point, 0.001f);
}

namespace testing {

typedef std::tuple<int /* trajectory_id */, std::string /* sensor_id */,
                   common::Time>
    CollatorOutput;

struct CollatorInput {
  static CollatorInput CreateImuData(int trajectory_id,
                                     const std::string& sensor_id, int time) {
    return CollatorInput{
        trajectory_id,
        MakeDispatchable(sensor_id, ImuData{common::FromUniversal(time)}),
        CollatorOutput{trajectory_id, sensor_id, common::FromUniversal(time)}};
  }
  static CollatorInput CreateTimedPointCloudData(int trajectory_id,
                                                 const std::string& sensor_id,
                                                 int time) {
    return CollatorInput{
        trajectory_id,
        MakeDispatchable(
            sensor_id,
            TimedPointCloudData{
                common::FromUniversal(time), Eigen::Vector3f::Zero(), {}}),
        CollatorOutput{trajectory_id, sensor_id, common::FromUniversal(time)}};
  }
  static CollatorInput CreateOdometryData(int trajectory_id,
                                          const std::string& sensor_id,
                                          int time) {
    return CollatorInput{
        trajectory_id,
        MakeDispatchable(sensor_id,
                         OdometryData{common::FromUniversal(time),
                                      transform::Rigid3d::Identity()}),
        CollatorOutput{trajectory_id, sensor_id, common::FromUniversal(time)}};
  }
  void MoveToCollator(CollatorInterface* collator) {
    collator->AddSensorData(trajectory_id, std::move(data));
  }

  const int trajectory_id;
  std::unique_ptr<Data> data;
  const CollatorOutput expected_output;
};

}  // namespace testing
}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_TEST_HELPERS_H_
