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

#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace sensor {

void Collator::AddTrajectory(
    const int trajectory_id,
    const std::unordered_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

void Collator::AddSensorData(const int trajectory_id,
                             const std::string& sensor_id,
                             std::unique_ptr<Data> data) {
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // namespace sensor
}  // namespace cartographer
