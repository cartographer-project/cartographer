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

#include "absl/strings/str_cat.h"

namespace cartographer {
namespace sensor {

metrics::Family<metrics::Counter>*
    TrajectoryCollator::collator_metrics_family_ =
        metrics::Family<metrics::Counter>::Null();

void TrajectoryCollator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
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
  auto* metric = GetOrCreateSensorMetric(data->GetSensorId(), trajectory_id);
  metric->Increment();
  trajectory_to_queue_.at(trajectory_id)
      .Add(std::move(queue_key), std::move(data));
}

void TrajectoryCollator::Flush() {
  for (auto& it : trajectory_to_queue_) {
    it.second.Flush();
  }
}

absl::optional<int> TrajectoryCollator::GetBlockingTrajectoryId() const {
  return absl::optional<int>();
}

void TrajectoryCollator::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  collator_metrics_family_ = family_factory->NewCounterFamily(
      "collator_input_total", "Sensor data received");
}

metrics::Counter* TrajectoryCollator::GetOrCreateSensorMetric(
    const std::string& sensor_id, int trajectory_id) {
  const std::string trajectory_id_str = absl::StrCat(trajectory_id);
  const std::string map_key = absl::StrCat(sensor_id, "/", trajectory_id_str);

  auto metrics_map_itr = metrics_map_.find(map_key);
  if (metrics_map_itr != metrics_map_.end()) {
    return metrics_map_itr->second;
  }

  LOG(INFO) << "Create metrics handler for key: " << map_key;
  auto new_counter = collator_metrics_family_->Add(
      {{"sensor_id", sensor_id}, {"trajectory_id", trajectory_id_str}});

  metrics_map_[map_key] = new_counter;
  return new_counter;
}

}  // namespace sensor
}  // namespace cartographer
