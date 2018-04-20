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

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace cartographer {
namespace common {
namespace testing {

ThreadPoolForTesting::ThreadPoolForTesting()
    : thread_([this]() { ThreadPoolForTesting::DoWork(); }) {}

ThreadPoolForTesting::~ThreadPoolForTesting() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(work_queue_.size(), 0);
  }
  thread_.join();
}

void ThreadPoolForTesting::Schedule(const std::function<void()> &work_item) {
  MutexLocker locker(&mutex_);
  idle_ = false;
  CHECK(running_);
  work_queue_.push_back(work_item);
}

void ThreadPoolForTesting::WaitUntilIdle() {
  for (;;) {
    {
      common::MutexLocker locker(&mutex_);
      if (locker.AwaitWithTimeout([this]() REQUIRES(mutex_) { return idle_; },
                                  common::FromSeconds(0.1))) {
        return;
      }
    }
  }
}

void ThreadPoolForTesting::DoWork() {
  for (;;) {
    std::function<void()> work_item;
    {
      MutexLocker locker(&mutex_);
      locker.AwaitWithTimeout(
          [this]()
              REQUIRES(mutex_) { return !work_queue_.empty() || !running_; },
          common::FromSeconds(0.1));
      if (!work_queue_.empty()) {
        work_item = work_queue_.front();
        work_queue_.pop_front();
      }
      if (!running_) {
        return;
      }
      if (work_queue_.empty() && !work_item) {
        idle_ = true;
      }
    }
    if (work_item) work_item();
  }
}

}  // namespace testing
}  // namespace common
}  // namespace cartographer
