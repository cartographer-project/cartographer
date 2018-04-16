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

#include <queue>

#include "cartographer/common/task.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class FakeThreadPool : public ThreadPoolInterface {
 public:
  void Schedule(const Task::WorkItem& work_item) override {
    work_queue_.push(work_item);
  }

  void RunNext() {
    ASSERT_GE(work_queue_.size(), 1);
    work_queue_.front()();
    work_queue_.pop();
  }

  bool IsEmpty() { return work_queue_.empty(); }

 private:
  std::queue<Task::WorkItem> work_queue_;
};

class TaskTest : public ::testing::Test {
 protected:
  FakeThreadPool* thread_pool() { return &thread_pool_; }
  FakeThreadPool thread_pool_;
};

TEST_F(TaskTest, RunTask) {
  Task a;
  ASSERT_EQ(a.GetState(), Task::IDLE);
  a.Dispatch(thread_pool());
  ASSERT_EQ(a.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_TRUE(thread_pool()->IsEmpty());
  ASSERT_EQ(a.GetState(), Task::COMPLETED);
}

TEST_F(TaskTest, RunTaskWithDependency) {
  Task a;
  Task b;
  b.AddDependency(&a);
  ASSERT_EQ(a.GetState(), Task::IDLE);
  ASSERT_EQ(b.GetState(), Task::IDLE);
  b.Dispatch(thread_pool());
  ASSERT_TRUE(thread_pool()->IsEmpty());
  a.Dispatch(thread_pool());
  thread_pool()->RunNext();
  ASSERT_EQ(a.GetState(), Task::COMPLETED);
  ASSERT_EQ(b.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_TRUE(thread_pool()->IsEmpty());
  ASSERT_EQ(b.GetState(), Task::COMPLETED);
}

TEST_F(TaskTest, RunTaskWithTwoDependency) {
  // a
  // |
  // v
  // b  c
  // |  |
  // | /
  //	|
  // v
  // d
  Task a;
  Task b;
  Task c;
  Task d;
  b.AddDependency(&a);
  d.AddDependency(&b);
  d.AddDependency(&c);
  d.Dispatch(thread_pool());
  ASSERT_EQ(d.GetState(), Task::DISPATCHED);
  ASSERT_TRUE(thread_pool()->IsEmpty());
  b.Dispatch(thread_pool());
  ASSERT_EQ(b.GetState(), Task::DISPATCHED);
  ASSERT_TRUE(thread_pool()->IsEmpty());
  c.Dispatch(thread_pool());
  ASSERT_EQ(c.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_TRUE(thread_pool()->IsEmpty());
  ASSERT_EQ(c.GetState(), Task::COMPLETED);
  ASSERT_EQ(d.GetState(), Task::DISPATCHED);
  a.Dispatch(thread_pool());
  ASSERT_EQ(a.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_EQ(a.GetState(), Task::COMPLETED);
  ASSERT_EQ(b.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_EQ(b.GetState(), Task::COMPLETED);
  ASSERT_EQ(d.GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  ASSERT_EQ(d.GetState(), Task::COMPLETED);
  ASSERT_TRUE(thread_pool()->IsEmpty());
}

TEST_F(TaskTest, RunWithCompletedDependency) {
  Task a;
  a.Dispatch(thread_pool());
  thread_pool()->RunNext();
  ASSERT_TRUE(thread_pool()->IsEmpty());
  ASSERT_EQ(a.GetState(), Task::COMPLETED);

  Task b;
  b.AddDependency(&a);
  ASSERT_EQ(b.GetState(), Task::IDLE);
  b.Dispatch(thread_pool());
  ASSERT_EQ(b.GetState(), Task::DISPATCHED);
  ASSERT_FALSE(thread_pool()->IsEmpty());
  thread_pool()->RunNext();
  ASSERT_TRUE(thread_pool()->IsEmpty());
  ASSERT_EQ(b.GetState(), Task::COMPLETED);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
