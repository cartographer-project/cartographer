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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_

#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

// Waits to see at least one data item for all sensor ids and dispatches data
// in merge-sorted order. Contrary to 'Collator', it does not wait for other
// trajectories.
// Also contrary to 'Collator', whose output is deterministic, the sequence in
// which data is dispatched is not sorted, so non-deterministic input sequences
// will result in non-deterministic output.
class TrajectoryCollator : public CollatorInterface {
 public:
  TrajectoryCollator() {}

  TrajectoryCollator(const TrajectoryCollator&) = delete;
  TrajectoryCollator& operator=(const TrajectoryCollator&) = delete;

  void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) override;

  void FinishTrajectory(int trajectory_id) override;

  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

  void Flush() override;

  absl::optional<int> GetBlockingTrajectoryId() const override;

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  metrics::Counter* GetOrCreateSensorMetric(const std::string& sensor_id,
                                            int trajectory_id);

  static cartographer::metrics::Family<metrics::Counter>*
      collator_metrics_family_;

  // Holds individual counters for each trajectory/sensor pair.
  absl::flat_hash_map<std::string, metrics::Counter*> metrics_map_;

  absl::flat_hash_map<int, OrderedMultiQueue> trajectory_to_queue_;

  // Map of trajectory ID to all associated QueueKeys.
  absl::flat_hash_map<int, std::vector<QueueKey>> trajectory_to_queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_TRAJECTORY_COLLATOR_H_
