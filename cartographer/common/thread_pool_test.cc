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

#include "cartographer/common/thread_pool.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class Receiver {
 public:
  void Receive(int number) {
    Mutex::Locker locker(&mutex_);
    received_numbers_.push_back(number);
  }

  void WaitForNumberSequence(const std::vector<int>& expected_numbers) {
    bool have_enough_numbers = false;
    while (!have_enough_numbers) {
      common::MutexLocker locker(&mutex_);
      have_enough_numbers = locker.AwaitWithTimeout(
          [this, &expected_numbers]() REQUIRES(mutex_) {
            return (received_numbers_.size() >= expected_numbers.size());
          },
          common::FromSeconds(0.1));
    }
    EXPECT_EQ(expected_numbers, received_numbers_);
  }

  std::vector<int> received_numbers_;
  Mutex mutex_;
};

TEST(ThreadPoolTest, RunTask) {
  ThreadPool pool(1);
  Receiver receiver;
  auto task = common::make_unique<Task>();
  task->SetWorkItem([&receiver]() { receiver.Receive(1); });
  pool.Schedule(std::move(task));
  receiver.WaitForNumberSequence({1});
}

TEST(ThreadPoolTest, RunWithDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_2 = common::make_unique<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_1 = common::make_unique<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  auto weak_task_1 = pool.Schedule(std::move(task_1));
  task_2->AddDependency(weak_task_1);
  pool.Schedule(std::move(task_2));
  receiver.WaitForNumberSequence({1, 2});
}

TEST(ThreadPoolTest, RunWithOutOfScopeDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_2 = common::make_unique<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  {
    auto task_1 = common::make_unique<Task>();
    task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
    auto weak_task_1 = pool.Schedule(std::move(task_1));
    task_2->AddDependency(weak_task_1);
  }
  pool.Schedule(std::move(task_2));
  receiver.WaitForNumberSequence({1, 2});
}

TEST(ThreadPoolTest, RunWithMultipleDependencies) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_1 = common::make_unique<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  auto task_2a = common::make_unique<Task>();
  task_2a->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_2b = common::make_unique<Task>();
  task_2b->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_3 = common::make_unique<Task>();
  task_3->SetWorkItem([&receiver]() { receiver.Receive(3); });
  /*          -> task_2a \
   *  task_1 /-> task_2b --> task_3
   */
  auto weak_task_1 = pool.Schedule(std::move(task_1));
  task_2a->AddDependency(weak_task_1);
  auto weak_task_2a = pool.Schedule(std::move(task_2a));
  task_3->AddDependency(weak_task_1);
  task_3->AddDependency(weak_task_2a);
  task_2b->AddDependency(weak_task_1);
  auto weak_task_2b = pool.Schedule(std::move(task_2b));
  task_3->AddDependency(weak_task_2b);
  pool.Schedule(std::move(task_3));
  receiver.WaitForNumberSequence({1, 2, 2, 3});
}

TEST(ThreadPoolTest, RunWithFinishedDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_1 = common::make_unique<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  auto task_2 = common::make_unique<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto weak_task_1 = pool.Schedule(std::move(task_1));
  task_2->AddDependency(weak_task_1);
  receiver.WaitForNumberSequence({1});
  pool.Schedule(std::move(task_2));
  receiver.WaitForNumberSequence({1, 2});
}

}  // namespace
}  // namespace common
}  // namespace cartographer
