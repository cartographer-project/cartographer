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
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/dispatchable.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
template <class QueueKey>
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue() {}
  ~OrderedMultiQueue() {
    for (auto& entry : queues_) {
      CHECK(entry.second.finished);
    }
  }
  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key, Callback callback) {
    CHECK_EQ(queues_.count(queue_key), 0);
    queues_[queue_key].callback = std::move(callback);
  }

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key) {
    auto it = queues_.find(queue_key);
    CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
    auto& queue = it->second;
    CHECK(!queue.finished);
    queue.finished = true;
    Dispatch();
  }

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data) {
    auto it = queues_.find(queue_key);
    if (it == queues_.end()) {
      LOG_EVERY_N(WARNING, 1000)
          << "Ignored data for queue: '" << queue_key << "'";
      return;
    }
    it->second.queue.Push(std::move(data));
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

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const {
    CHECK(!queues_.empty());
    return blocker_;
  }

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch() {
    while (true) {
      const Data* next_data = nullptr;
      Queue* next_queue = nullptr;
      QueueKey next_queue_key;
      for (auto it = queues_.begin(); it != queues_.end();) {
        const Data* data = it->second.queue.template Peek<Data>();
        if (data == nullptr) {
          if (it->second.finished) {
            queues_.erase(it++);
            continue;
          }
          CannotMakeProgress(it->first);
          return;
        }
        if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
          next_data = data;
          next_queue = &it->second;
          next_queue_key = it->first;
        }
        CHECK_LE(last_dispatched_time_, next_data->GetTime())
            << "Non-sorted data added to queue: '" << it->first << "'";
        ++it;
      }
      if (next_data == nullptr) {
        CHECK(queues_.empty());
        return;
      }

      // If we haven't dispatched any data for this trajectory yet, fast forward
      // all queues of this trajectory until a common start time has been
      // reached.
      const common::Time common_start_time =
          GetCommonStartTime(next_queue_key.trajectory_id);

      if (next_data->GetTime() >= common_start_time) {
        // Happy case, we are beyond the 'common_start_time' already.
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(next_queue->queue.Pop());
      } else if (next_queue->queue.Size() < 2) {
        if (!next_queue->finished) {
          // We cannot decide whether to drop or dispatch this yet.
          CannotMakeProgress(next_queue_key);
          return;
        }
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(next_queue->queue.Pop());
      } else {
        // We take a peek at the time after next data. If it also is not beyond
        // 'common_start_time' we drop 'next_data', otherwise we just found the
        // first packet to dispatch from this queue.
        std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
        if (next_queue->queue.template Peek<Data>()->GetTime() >
            common_start_time) {
          last_dispatched_time_ = next_data->GetTime();
          next_queue->callback(std::move(next_data_owner));
        }
      }
    }
  }

  void CannotMakeProgress(const QueueKey& queue_key) {
    blocker_ = queue_key;
    for (auto& entry : queues_) {
      if (entry.second.queue.Size() > kMaxQueueSize) {
        LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
        return;
      }
    }
  }

  common::Time GetCommonStartTime(const int trajectory_id) {
    auto emplace_result = common_start_time_per_trajectory_.emplace(
        trajectory_id, common::Time::min());
    common::Time& common_start_time = emplace_result.first->second;
    if (emplace_result.second) {
      for (auto& entry : queues_) {
        if (entry.first.trajectory_id == trajectory_id) {
          common_start_time =
              std::max(common_start_time,
                       entry.second.queue.template Peek<Data>()->GetTime());
        }
      }
      LOG(INFO) << "All sensor data for trajectory " << trajectory_id
                << " is available starting at '" << common_start_time << "'.";
    }
    return common_start_time;
  }

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
