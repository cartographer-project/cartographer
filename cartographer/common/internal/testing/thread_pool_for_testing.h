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
#include <map>
#include <thread>

#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"

namespace cartographer {
namespace common {
namespace testing {

class ThreadPoolForTesting : public ThreadPoolInterface {
 public:
  ThreadPoolForTesting();
  ~ThreadPoolForTesting();

  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)
      EXCLUDES(mutex_) override;

  void WaitUntilIdle();

 private:
  friend class Task;

  void DoWork();

  void NotifyDependenciesCompleted(Task* task) EXCLUDES(mutex_) override;

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  bool idle_ GUARDED_BY(mutex_) = true;
  std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_);
  std::map<Task*, std::shared_ptr<Task>> tasks_not_ready_ GUARDED_BY(mutex_);
  std::thread thread_ GUARDED_BY(mutex_);
};

}  // namespace testing
}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_INTERNAL_TESTING_THREAD_POOL_FOR_TESTING_H_
