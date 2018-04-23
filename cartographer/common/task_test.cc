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

#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class MockThreadPool : public ThreadPoolInterface {
 public:
  MOCK_METHOD1(NotifyReady, void(Task*));
  MOCK_METHOD1(Schedule, void(const std::function<void()>&));
  // Work-around gmock's unique_ptr limitation.
  MOCK_METHOD1(ScheduleWhenReadyWithPtr, std::weak_ptr<Task>(Task*));
  std::weak_ptr<Task> ScheduleWhenReady(std::unique_ptr<Task> task) override {
    return ScheduleWhenReadyWithPtr(task.get());
  }
};

class MockCallback {
 public:
  MOCK_METHOD0(Run, void());
};

TEST(TaskTest, RunTask) {
  MockThreadPool thread_pool;
  Task a;
  MockCallback callback;
  a.SetWorkItem([&callback]() { callback.Run(); });
  EXPECT_EQ(a.GetState(), Task::NEW);
  EXPECT_CALL(thread_pool, NotifyReady(&a)).Times(1);
  a.NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a.GetState(), Task::READY);
  EXPECT_CALL(callback, Run()).Times(1);
  a.Execute();
  EXPECT_EQ(a.GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunTaskWithDependency) {
  MockThreadPool thread_pool;
  auto a = std::make_shared<Task>();
  auto b = std::make_shared<Task>();
  MockCallback callback_a;
  a->SetWorkItem([&callback_a]() { callback_a.Run(); });
  MockCallback callback_b;
  b->SetWorkItem([&callback_b]() { callback_b.Run(); });
  b->AddDependency(a);
  EXPECT_EQ(a->GetState(), Task::NEW);
  EXPECT_EQ(b->GetState(), Task::NEW);
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(thread_pool, NotifyReady(a.get())).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(b.get())).Times(1);
  }
  b->NotifyWhenReady(&thread_pool);
  EXPECT_EQ(b->GetState(), Task::DISPATCHED);
  a->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a->GetState(), Task::READY);
  EXPECT_CALL(callback_a, Run()).Times(1);
  a->Execute();
  ASSERT_EQ(b->GetState(), Task::READY);
  EXPECT_CALL(callback_b, Run()).Times(1);
  b->Execute();
  EXPECT_EQ(a->GetState(), Task::COMPLETED);
  EXPECT_EQ(b->GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunTaskWithTwoDependency) {
  MockThreadPool thread_pool;
  /*         c \
   *  a -->  b --> d
   */
  auto a = std::make_shared<Task>();
  auto b = std::make_shared<Task>();
  auto c = std::make_shared<Task>();
  auto d = std::make_shared<Task>();
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(thread_pool, NotifyReady(c.get())).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(a.get())).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(b.get())).Times(1);
    EXPECT_CALL(thread_pool, NotifyReady(d.get())).Times(1);
  }
  b->AddDependency(a);
  d->AddDependency(b);
  d->AddDependency(c);
  d->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(d->GetState(), Task::DISPATCHED);
  b->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(b->GetState(), Task::DISPATCHED);
  c->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(c->GetState(), Task::READY);
  c->Execute();
  EXPECT_EQ(c->GetState(), Task::COMPLETED);
  ASSERT_EQ(d->GetState(), Task::DISPATCHED);
  a->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a->GetState(), Task::READY);
  a->Execute();
  EXPECT_EQ(a->GetState(), Task::COMPLETED);
  ASSERT_EQ(b->GetState(), Task::READY);
  b->Execute();
  EXPECT_EQ(b->GetState(), Task::COMPLETED);
  ASSERT_EQ(d->GetState(), Task::READY);
  d->Execute();
  EXPECT_EQ(d->GetState(), Task::COMPLETED);
}

TEST(TaskTest, RunWithCompletedDependency) {
  MockThreadPool thread_pool;
  auto a = std::make_shared<Task>();
  EXPECT_CALL(thread_pool, NotifyReady(a.get())).Times(1);
  a->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(a->GetState(), Task::READY);
  a->Execute();
  EXPECT_EQ(a->GetState(), Task::COMPLETED);
  auto b = std::make_shared<Task>();
  EXPECT_CALL(thread_pool, NotifyReady(b.get())).Times(1);
  b->AddDependency(a);
  EXPECT_EQ(b->GetState(), Task::NEW);
  b->NotifyWhenReady(&thread_pool);
  ASSERT_EQ(b->GetState(), Task::READY);
  b->Execute();
  EXPECT_EQ(b->GetState(), Task::COMPLETED);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
