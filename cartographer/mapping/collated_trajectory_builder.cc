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

#include "cartographer/mapping/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    sensor::Collator* const sensor_collator, const int trajectory_id,
    const std::unordered_set<string>& expected_sensor_ids,
    std::unique_ptr<GlobalTrajectoryBuilderInterface>
        wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_ids,
      [this](const string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

CollatedTrajectoryBuilder::~CollatedTrajectoryBuilder() {}

const Submaps* CollatedTrajectoryBuilder::submaps() const {
  return wrapped_trajectory_builder_->submaps();
}

const TrajectoryBuilder::PoseEstimate&
CollatedTrajectoryBuilder::pose_estimate() const {
  return wrapped_trajectory_builder_->pose_estimate();
}

void CollatedTrajectoryBuilder::AddSensorData(
    const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, sensor_id, std::move(data));
}

void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->time);

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  switch (data->type) {
    case sensor::Data::Type::kImu:
      wrapped_trajectory_builder_->AddImuData(data->time,
                                              data->imu.linear_acceleration,
                                              data->imu.angular_velocity);
      return;

    case sensor::Data::Type::kRangefinder:
      wrapped_trajectory_builder_->AddRangefinderData(
          data->time, data->rangefinder.origin, data->rangefinder.ranges);
      return;

    case sensor::Data::Type::kOdometer:
      wrapped_trajectory_builder_->AddOdometerData(data->time,
                                                   data->odometer_pose);
      return;
  }
  LOG(FATAL);
}

}  // namespace mapping
}  // namespace cartographer
