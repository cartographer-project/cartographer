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

  void WaitForNumberSequence(const std::list<int>& expected_numbers) {
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

  std::list<int> received_numbers_;
  Mutex mutex_;
};

TEST(ThreadPoolTest, RunTask) {
  ThreadPool pool(1);
  Receiver receiver;
  auto task = std::make_shared<Task>();
  task->SetWorkItem([&receiver]() { receiver.Receive(1); });
  pool.ScheduleWhenReady(task);
  receiver.WaitForNumberSequence({1});
}

TEST(ThreadPoolTest, RunWithDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  task_2->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_2);
  pool.ScheduleWhenReady(task_1);
  receiver.WaitForNumberSequence({1, 2});
}

TEST(ThreadPoolTest, RunWithOutOfScopeDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  {
    auto task_1 = std::make_shared<Task>();
    task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
    pool.ScheduleWhenReady(task_1);
    task_2->AddDependency(task_1.get());
    // task_1 goes out of scope.
  }
  pool.ScheduleWhenReady(task_2);
  receiver.WaitForNumberSequence({1, 2});
}

TEST(ThreadPoolTest, RunWithMultipleDependencies) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  auto task_2a = std::make_shared<Task>();
  task_2a->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_2b = std::make_shared<Task>();
  task_2b->SetWorkItem([&receiver]() { receiver.Receive(2); });
  auto task_3 = std::make_shared<Task>();
  task_3->SetWorkItem([&receiver]() { receiver.Receive(3); });
  /*          -> task_2a \
   *  task_1 /-> task_2b --> task_3
   */
  task_3->AddDependency(task_2a.get());
  task_3->AddDependency(task_2b.get());
  task_2a->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_1);
  pool.ScheduleWhenReady(task_2a);
  task_3->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_3);
  task_2b->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_2b);
  receiver.WaitForNumberSequence({1, 2, 2, 3});
}

TEST(ThreadPoolTest, RunWithFinishedDependency) {
  ThreadPool pool(2);
  Receiver receiver;
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });
  task_2->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_1);
  receiver.WaitForNumberSequence({1});
  pool.ScheduleWhenReady(task_2);
  receiver.WaitForNumberSequence({1, 2});
}

}  // namespace
}  // namespace common
}  // namespace cartographer
