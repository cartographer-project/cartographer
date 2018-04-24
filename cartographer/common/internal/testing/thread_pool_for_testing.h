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

#ifndef CARTOGRAPHER_COMMON_INTERNAL_TESTING_THREAD_POOL_FOR_TESTING_H_
#define CARTOGRAPHER_COMMON_INTERNAL_TESTING_THREAD_POOL_FOR_TESTING_H_

#include <deque>
#include <functional>
#include <thread>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"

namespace cartographer {
namespace common {
namespace testing {

class ThreadPoolForTesting : public ThreadPoolInterface {
 public:
  ThreadPoolForTesting();
  ~ThreadPoolForTesting();

  void NotifyDependenciesCompleted(Task* task) EXCLUDES(mutex_) override {
    LOG(FATAL) << "not implemented";
  }

  void Schedule(const std::function<void()>& work_item) override;
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)
      EXCLUDES(mutex_) override {
    LOG(FATAL) << "not implemented";
  }

  void WaitUntilIdle();

 private:
  void DoWork();

  std::thread thread_ GUARDED_BY(mutex_);
  bool running_ GUARDED_BY(mutex_) = true;
  bool idle_ GUARDED_BY(mutex_) = true;
  std::deque<std::function<void()>> work_queue_ GUARDED_BY(mutex_);
  Mutex mutex_;
};

}  // namespace testing
}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_INTERNAL_TESTING_THREAD_POOL_FOR_TESTING_H_
