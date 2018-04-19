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

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class ThreadPoolForTesting : public ThreadPool {
 public:
  ThreadPoolForTesting() : ThreadPool(1) {}

  void WaitForAllForTesting() override { ThreadPool::WaitForAllForTesting(); }
};

class Mock {
 public:
  MOCK_METHOD1(Run, void(int));
};

TEST(ThreadPoolTest, RunTask) {
  ThreadPoolForTesting pool;
  Mock mock;
  auto task = std::make_shared<Task>();
  task->SetWorkItem([&mock]() { mock.Run(1); });
  EXPECT_CALL(mock, Run(1)).Times(1);
  pool.ScheduleWhenReady(task);
  pool.WaitForAllForTesting();
}

TEST(ThreadPoolTest, RunWithDependency) {
  ThreadPoolForTesting pool;
  Mock mock;
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&mock]() { mock.Run(2); });
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&mock]() { mock.Run(1); });
  task_2->AddDependency(task_1.get());
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(mock, Run(1)).Times(1);
    EXPECT_CALL(mock, Run(2)).Times(1);
  }
  pool.ScheduleWhenReady(task_2);
  pool.ScheduleWhenReady(task_1);
  pool.WaitForAllForTesting();
}

TEST(ThreadPoolTest, RunWithOutOfScopeDependency) {
  ThreadPoolForTesting pool;
  Mock mock;
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(mock, Run(1)).Times(1);
    EXPECT_CALL(mock, Run(2)).Times(1);
  }
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&mock]() { mock.Run(2); });
  {
    auto task_1 = std::make_shared<Task>();
    task_1->SetWorkItem([&mock]() { mock.Run(1); });
    pool.ScheduleWhenReady(task_1);
    task_2->AddDependency(task_1.get());
    // task_1 goes out of scope.
  }
  pool.ScheduleWhenReady(task_2);
  pool.WaitForAllForTesting();
}

TEST(ThreadPoolTest, RunWithMultipleDependencies) {
  ThreadPoolForTesting pool;
  Mock mock;
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&mock]() { mock.Run(1); });
  auto task_2a = std::make_shared<Task>();
  task_2a->SetWorkItem([&mock]() { mock.Run(2); });
  auto task_2b = std::make_shared<Task>();
  task_2b->SetWorkItem([&mock]() { mock.Run(2); });
  auto task_3 = std::make_shared<Task>();
  task_3->SetWorkItem([&mock]() { mock.Run(3); });
  /*          -> task_2a \
   *  task_1 /-> task_2b --> task_3
   */
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(mock, Run(1)).Times(1);
    EXPECT_CALL(mock, Run(2)).Times(2);
    EXPECT_CALL(mock, Run(3)).Times(1);
  }
  task_3->AddDependency(task_2a.get());
  task_3->AddDependency(task_2b.get());
  task_2a->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_1);
  pool.ScheduleWhenReady(task_2a);
  task_3->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_3);
  task_2b->AddDependency(task_1.get());
  pool.ScheduleWhenReady(task_2b);
  pool.WaitForAllForTesting();
}

TEST(ThreadPoolTest, RunWithFinishedDependency) {
  ThreadPoolForTesting pool;
  Mock mock;
  auto task_1 = std::make_shared<Task>();
  task_1->SetWorkItem([&mock]() { mock.Run(1); });
  auto task_2 = std::make_shared<Task>();
  task_2->SetWorkItem([&mock]() { mock.Run(2); });
  task_2->AddDependency(task_1.get());
  EXPECT_CALL(mock, Run(1)).Times(1);
  pool.ScheduleWhenReady(task_1);
  pool.WaitForAllForTesting();
  EXPECT_CALL(mock, Run(2)).Times(1);
  pool.ScheduleWhenReady(task_2);
  pool.WaitForAllForTesting();
}

}  // namespace
}  // namespace common
}  // namespace cartographer
