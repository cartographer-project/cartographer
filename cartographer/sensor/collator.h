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

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/ordered_multi_queue.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

class Collator {
 public:
  using Callback = std::function<void(const string&, std::unique_ptr<Data>)>;

  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  void AddTrajectory(const int trajectory_id,
                     const std::unordered_set<string>& expected_sensor_ids,
                     const Callback callback) {
    for (const auto& sensor_id : expected_sensor_ids) {
      const auto queue_key = QueueKey{trajectory_id, sensor_id};
      queue_.AddQueue(queue_key,
                      [callback, sensor_id](std::unique_ptr<Data> data) {
                        callback(sensor_id, std::move(data));
                      });
      queue_keys_[trajectory_id].push_back(queue_key);
    }
  }

  // Marks 'trajectory_id' as finished.
  void FinishTrajectory(const int trajectory_id) {
    for (const auto& queue_key : queue_keys_[trajectory_id]) {
      queue_.MarkQueueAsFinished(queue_key);
    }
  }

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'sensor_id' must be added in time
  // order.
  void AddSensorData(const int trajectory_id, const string& sensor_id,
                     std::unique_ptr<Data> data) {
    queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
  }

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  void Flush() {
    queue_.Flush();
  }

  // Returns the number of packets associated with 'trajectory_id' that are
  // available for processing.
  int num_available_packets(const int trajectory_id) {
    int num = std::numeric_limits<int>::max();
    for (const auto& queue_key : queue_keys_[trajectory_id]) {
      num = std::min(num, queue_.num_available(queue_key));
    }
    return num;
  }

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_
