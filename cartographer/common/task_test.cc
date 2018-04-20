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

#include "cartographer/common/task.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class MockThreadPool : public ThreadPoolInterface {
 public:
  MOCK_METHOD1(NotifyReady, void(Task*));
  MOCK_METHOD1(Schedule, void(const std::function<void()>&));
  MOCK_METHOD1(ScheduleWhenReady, std::shared_ptr<Task>(std::shared_ptr<Task>));
};

TEST(TaskTest, RunTask) {
  MockThreadPool thread_pool;
  Task a;
  EXPECT_EQ(a.GetState(), Task::NEW);
  EXPECT_CALL(thread_pool, NotifyReady(&a)).Times(1);
  a.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a.GetState(), Task::READY);
  a.Execute();
  EXPECT_EQ(a.GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunTaskWithDependency) {
  MockThreadPool thread_pool;
  Task a;
  Task b;
  b.AddDependency(&a);
  EXPECT_EQ(a.GetState(), Task::NEW);
  EXPECT_EQ(b.GetState(), Task::NEW);
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(thread_pool, NotifyReady(&a)).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(&b)).Times(1);
  }
  b.NotifyWhenReady(&thread_pool);
  EXPECT_EQ(b.GetState(), Task::DISPATCHED);
  a.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a.GetState(), Task::READY);
  a.Execute();
  ASSERT_EQ(b.GetState(), Task::READY);
  b.Execute();
  EXPECT_EQ(a.GetState(), Task::COMPLETED);
  EXPECT_EQ(b.GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunTaskWithTwoDependency) {
  MockThreadPool thread_pool;
  /*         c \
   *  a -->  b --> d
   */
  Task a;
  Task b;
  Task c;
  Task d;
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(thread_pool, NotifyReady(&c)).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(&a)).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(&b)).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(&d)).Times(1);
  }
  b.AddDependency(&a);
  d.AddDependency(&b);
  d.AddDependency(&c);
  d.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(d.GetState(), Task::DISPATCHED);
  b.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(b.GetState(), Task::DISPATCHED);
  c.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(c.GetState(), Task::READY);
  c.Execute();
  EXPECT_EQ(c.GetState(), Task::COMPLETED);
  ASSERT_EQ(d.GetState(), Task::DISPATCHED);
  a.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a.GetState(), Task::READY);
  a.Execute();
  EXPECT_EQ(a.GetState(), Task::COMPLETED);
  ASSERT_EQ(b.GetState(), Task::READY);
  b.Execute();
  EXPECT_EQ(b.GetState(), Task::COMPLETED);
  ASSERT_EQ(d.GetState(), Task::READY);
  d.Execute();
  EXPECT_EQ(d.GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunWithCompletedDependency) {
  MockThreadPool thread_pool;
  Task a;
  EXPECT_CALL(thread_pool, NotifyReady(&a)).Times(1);
  a.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a.GetState(), Task::READY);
  a.Execute();
  EXPECT_EQ(a.GetState(), Task::COMPLETED);
  Task b;
  EXPECT_CALL(thread_pool, NotifyReady(&b)).Times(1);
  b.AddDependency(&a);
  EXPECT_EQ(b.GetState(), Task::NEW);
  b.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(b.GetState(), Task::READY);
  b.Execute();
  EXPECT_EQ(b.GetState(), Task::COMPLETED);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
