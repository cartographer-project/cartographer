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

#include "cartographer/sensor/internal/trajectory_collator.h"

namespace cartographer {
namespace sensor {

void TrajectoryCollator::AddTrajectory(
    const int trajectory_id,
    const std::unordered_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  CHECK_EQ(trajectory_to_queue_.count(trajectory_id), 0);
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    trajectory_to_queue_[trajectory_id].AddQueue(
        queue_key, [callback, sensor_id](std::unique_ptr<Data> data) {
          callback(sensor_id, std::move(data));
        });
    trajectory_to_queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void TrajectoryCollator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : trajectory_to_queue_keys_[trajectory_id]) {
    trajectory_to_queue_.at(trajectory_id).MarkQueueAsFinished(queue_key);
  }
}

void TrajectoryCollator::AddSensorData(const int trajectory_id,
                                       std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  trajectory_to_queue_.at(trajectory_id)
      .Add(std::move(queue_key), std::move(data));
}

void TrajectoryCollator::Flush() {
  for (auto& it : trajectory_to_queue_) {
    it.second.Flush();
  }
}

common::optional<int> TrajectoryCollator::GetBlockingTrajectoryId() const {
  return common::optional<int>();
}

}  // namespace sensor
}  // namespace cartographer
