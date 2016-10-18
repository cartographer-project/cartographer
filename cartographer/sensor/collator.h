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
#include "cartographer/common/ordered_multi_queue.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/sensor_packet_period_histogram_builder.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

struct CollatorQueueKey {
  int trajectory_id;
  string sensor_id;

  bool operator<(const CollatorQueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

inline std::ostream& operator<<(std::ostream& out,
                                const CollatorQueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

class Collator {
 public:
  using Callback = std::function<void(int64, std::unique_ptr<Data>)>;

  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  void AddTrajectory(const int trajectory_id,
                     const std::unordered_set<string>& expected_sensor_ids,
                     const Callback callback) {
    for (const auto& sensor_id : expected_sensor_ids) {
      const auto queue_key = CollatorQueueKey{trajectory_id, sensor_id};
      queue_.AddQueue(queue_key, [callback](std::unique_ptr<Value> value) {
        callback(value->timestamp, std::move(value->sensor_data));
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

  // Adds 'sensor_data' for 'trajectory_id' to be collated. 'sensor_data' must
  // contain valid sensor data. Sensor packets with matching 'sensor_id' must be
  // added in time order.
  void AddSensorData(const int trajectory_id, const int64 timestamp,
                     const string& sensor_id,
                     std::unique_ptr<Data> sensor_data) {
    sensor_packet_period_histogram_builder_.Add(trajectory_id, timestamp,
                                                sensor_id);
    queue_.Add(
        CollatorQueueKey{trajectory_id, sensor_id}, timestamp,
        common::make_unique<Value>(Value{timestamp, std::move(sensor_data)}));
  }

  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  void Flush() {
    queue_.Flush();
    sensor_packet_period_histogram_builder_.LogHistogramsAndClear();
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
  struct Value {
    int64 timestamp;
    std::unique_ptr<Data> sensor_data;
  };

  // Queue keys are a pair of trajectory ID and sensor identifier.
  common::OrderedMultiQueue<CollatorQueueKey, int64, Value> queue_;

  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<CollatorQueueKey>> queue_keys_;
  sensor::SensorPacketPeriodHistogramBuilder
      sensor_packet_period_histogram_builder_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_
