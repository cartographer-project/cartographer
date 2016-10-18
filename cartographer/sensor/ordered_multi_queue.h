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

#ifndef CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

#include <algorithm>
#include <map>
#include <memory>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

// Number of items that can be queued up before we LOG(WARNING).
const int kMaxQueueSize = 500;

struct QueueKey {
  int trajectory_id;
  string sensor_id;

  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. This class is thread-compatible.
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(common::Time, std::unique_ptr<Data>)>;

  // Will wait to see at least one value for each 'expected_queue_keys' before
  // dispatching the next smallest value across all queues.
  OrderedMultiQueue() {}
  ~OrderedMultiQueue() {}

  void AddQueue(const QueueKey& queue_key, Callback callback) {
    CHECK(FindOrNull(queue_key) == nullptr);
    queues_[queue_key].callback = callback;
  }

  void MarkQueueAsFinished(const QueueKey& queue_key) {
    auto& queue = FindOrDie(queue_key);
    CHECK(!queue.finished);
    queue.finished = true;
    Dispatch();
  }

  bool HasQueue(const QueueKey& queue_key) {
    return queues_.count(queue_key) != 0;
  }

  void Add(const QueueKey& queue_key, const common::Time& sort_key,
           std::unique_ptr<Data> value) {
    auto* queue = FindOrNull(queue_key);
    if (queue == nullptr) {
      // TODO(damonkohler): This will not work for every value of "queue_key".
      LOG_EVERY_N(WARNING, 1000) << "Ignored value for queue: '" << queue_key
                                 << "'";
      return;
    }
    queue->queue.Push(
        common::make_unique<KeyValuePair>(sort_key, std::move(value)));
    Dispatch();
  }

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush() {
    std::vector<QueueKey> unfinished_queues;
    for (auto& entry : queues_) {
      if (!entry.second.finished) {
        unfinished_queues.push_back(entry.first);
      }
    }
    for (auto& unfinished_queue : unfinished_queues) {
      MarkQueueAsFinished(unfinished_queue);
    }
  }

  // Returns the number of available values associated with 'queue_key'.
  int num_available(const QueueKey& queue_key) {
    return FindOrDie(queue_key).queue.Size();
  }

 private:
  struct KeyValuePair {
    KeyValuePair(const common::Time& sort_key, std::unique_ptr<Data> value)
        : sort_key(sort_key), value(std::move(value)) {}
    common::Time sort_key;
    std::unique_ptr<Data> value;
  };

  struct Queue {
    common::BlockingQueue<std::unique_ptr<KeyValuePair>> queue;
    Callback callback;
    bool finished = false;
  };

  // Returns the queue with 'key' or LOG(FATAL).
  Queue& FindOrDie(const QueueKey& key) {
    auto it = queues_.find(key);
    CHECK(it != queues_.end()) << "Did not find '" << key << "'.";
    return it->second;
  }

  // Returns the queue with 'key' or nullptr.
  Queue* FindOrNull(const QueueKey& key) {
    auto it = queues_.find(key);
    if (it == queues_.end()) {
      return nullptr;
    }
    return &it->second;
  }

  void Dispatch() {
    while (true) {
      Queue* next_queue = nullptr;
      const KeyValuePair* next_key_value_pair = nullptr;
      for (auto it = queues_.begin(); it != queues_.end();) {
        auto& queue = it->second.queue;
        const auto* key_value_pair = queue.template Peek<KeyValuePair>();
        if (key_value_pair == nullptr) {
          if (it->second.finished) {
            queues_.erase(it++);
            continue;
          }
          CannotMakeProgress();
          return;
        }
        if (next_key_value_pair == nullptr ||
            std::forward_as_tuple(key_value_pair->sort_key, it->first) <
                std::forward_as_tuple(next_key_value_pair->sort_key,
                                      it->first)) {
          next_key_value_pair = key_value_pair;
          next_queue = &it->second;
        }
        CHECK_LE(last_dispatched_key_, next_key_value_pair->sort_key)
            << "Non-sorted values added to queue: '" << it->first << "'";
        ++it;
      }
      if (next_key_value_pair == nullptr) {
        CHECK(queues_.empty());
        return;
      }
      last_dispatched_key_ = next_key_value_pair->sort_key;
      next_queue->callback(last_dispatched_key_,
                           std::move(next_queue->queue.Pop()->value));
    }
  }

  // Called when not all necessary queues are filled to dispatch messages.
  void CannotMakeProgress() {
    for (auto& entry : queues_) {
      LOG_IF_EVERY_N(WARNING, entry.second.queue.Size() > kMaxQueueSize, 60)
          << "Queue " << entry.first << " exceeds maximum size.";
    }
  }

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_key_ = common::Time::min();

  std::map<QueueKey, Queue> queues_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
