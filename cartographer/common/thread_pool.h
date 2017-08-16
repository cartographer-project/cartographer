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

#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <thread>
#include <vector>

#include "cartographer/common/mutex.h"

namespace cartographer {
namespace common {

// A fixed number of threads working on a work queue of work items. Adding a
// new work item does not block, and will be executed by a background thread
// eventually. The queue must be empty before calling the destructor. The thread
// pool will then wait for the currently executing work items to finish and then
// destroy the threads.
class ThreadPool {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void Schedule(const std::function<void()>& work_item);

 private:
  void DoWork();

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::function<void()>> work_queue_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_
