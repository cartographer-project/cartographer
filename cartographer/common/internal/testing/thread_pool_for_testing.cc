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

#include "cartographer/common/internal/testing/thread_pool_for_testing.h"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "absl/memory/memory.h"
#include "cartographer/common/task.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {
namespace testing {

ThreadPoolForTesting::ThreadPoolForTesting()
    : thread_([this]() { ThreadPoolForTesting::DoWork(); }) {}

ThreadPoolForTesting::~ThreadPoolForTesting() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(task_queue_.size(), 0);
    CHECK_EQ(tasks_not_ready_.size(), 0);
  }
  thread_.join();
}

void ThreadPoolForTesting::NotifyDependenciesCompleted(Task* task) {
  absl::MutexLock locker(&mutex_);
  CHECK(running_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);
  tasks_not_ready_.erase(it);
}

std::weak_ptr<Task> ThreadPoolForTesting::Schedule(std::unique_ptr<Task> task) {
  std::shared_ptr<Task> shared_task;
  {
    absl::MutexLock locker(&mutex_);
    idle_ = false;
    CHECK(running_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
    CHECK(insert_result.second) << "ScheduleWhenReady called twice";
    shared_task = insert_result.first->second;
  }
  SetThreadPool(shared_task.get());
  return shared_task;
}

void ThreadPoolForTesting::WaitUntilIdle() {
  const auto predicate = [this]()
                             EXCLUSIVE_LOCKS_REQUIRED(mutex_) { return idle_; };
  for (;;) {
    {
      absl::MutexLock locker(&mutex_);
      if (mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(0.1)))) {
        return;
      }
    }
  }
}

void ThreadPoolForTesting::DoWork() {
  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return !task_queue_.empty() || !running_;
  };
  for (;;) {
    std::shared_ptr<Task> task;
    {
      absl::MutexLock locker(&mutex_);
      mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                              absl::FromChrono(common::FromSeconds(0.1)));
      if (!task_queue_.empty()) {
        task = task_queue_.front();
        task_queue_.pop_front();
      }
      if (!running_) {
        return;
      }
      if (tasks_not_ready_.empty() && task_queue_.empty() && !task) {
        idle_ = true;
      }
    }
    if (task) Execute(task.get());
  }
}

}  // namespace testing
}  // namespace common
}  // namespace cartographer
