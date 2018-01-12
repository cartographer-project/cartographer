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

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

// Waits to see at least one data item for all sensor ids on all unfinished
// trajectories and dispatches data in merge-sorted order.
class Collator : public CollatorInterface {
 public:
  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<std::string>& expected_sensor_ids,
                     const Callback& callback) override;

  void FinishTrajectory(int trajectory_id) override;

  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

  void Flush() override;

  int GetBlockingTrajectoryId() const override;

 private:
  struct QueueKey {
    int trajectory_id;
    std::string sensor_id;

    bool operator<(const QueueKey& other) const {
      return std::forward_as_tuple(trajectory_id, sensor_id) <
             std::forward_as_tuple(other.trajectory_id, other.sensor_id);
    }

    friend std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
      return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
    }
  };

  OrderedMultiQueue<QueueKey> queue_;

  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_
