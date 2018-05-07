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
#include <queue>

#include "cartographer/common/make_unique.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

class MockCallback {
 public:
  MOCK_METHOD0(Run, void());
};

class FakeThreadPool : public ThreadPoolInterface {
 public:
  void NotifyDependenciesCompleted(Task* task) override {
    auto it = tasks_not_ready_.find(task);
    ASSERT_NE(it, tasks_not_ready_.end());
    task_queue_.push_back(it->second);
    tasks_not_ready_.erase(it);
  }

  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override {
    std::shared_ptr<Task> shared_task;
    auto it =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
    EXPECT_TRUE(it.second);
    shared_task = it.first->second;
    SetThreadPool(shared_task.get());
    return shared_task;
  }

  void RunNext() {
    ASSERT_GE(task_queue_.size(), 1);
    Execute(task_queue_.front().get());
    task_queue_.pop_front();
  }

  bool IsEmpty() { return task_queue_.empty(); }

 private:
  std::deque<std::shared_ptr<Task>> task_queue_;
  std::map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

class TaskTest : public ::testing::Test {
 protected:
  FakeThreadPool* thread_pool() { return &thread_pool_; }
  FakeThreadPool thread_pool_;
};

TEST_F(TaskTest, RunTask) {
  auto a = make_unique<Task>();
  MockCallback callback;
  a->SetWorkItem([&callback]() { callback.Run(); });
  EXPECT_EQ(a->GetState(), Task::NEW);
  auto shared_a = thread_pool()->Schedule(std::move(a)).lock();
  EXPECT_NE(shared_a, nullptr);
  EXPECT_EQ(shared_a->GetState(), Task::DEPENDENCIES_COMPLETED);
  EXPECT_CALL(callback, Run()).Times(1);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_a->GetState(), Task::COMPLETED);
  EXPECT_TRUE(thread_pool()->IsEmpty());
}

TEST_F(TaskTest, RunTaskWithDependency) {
  auto a = make_unique<Task>();
  auto b = make_unique<Task>();
  MockCallback callback_a;
  a->SetWorkItem([&callback_a]() { callback_a.Run(); });
  MockCallback callback_b;
  b->SetWorkItem([&callback_b]() { callback_b.Run(); });
  EXPECT_EQ(a->GetState(), Task::NEW);
  EXPECT_EQ(b->GetState(), Task::NEW);
  {
    ::testing::InSequence dummy;
    EXPECT_CALL(callback_a, Run()).Times(1);
    EXPECT_CALL(callback_b, Run()).Times(1);
  }
  auto shared_a = thread_pool()->Schedule(std::move(a)).lock();
  EXPECT_NE(shared_a, nullptr);
  b->AddDependency(shared_a);
  auto shared_b = thread_pool()->Schedule(std::move(b)).lock();
  EXPECT_NE(shared_b, nullptr);
  EXPECT_EQ(shared_b->GetState(), Task::DISPATCHED);
  EXPECT_EQ(shared_a->GetState(), Task::DEPENDENCIES_COMPLETED);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_b->GetState(), Task::DEPENDENCIES_COMPLETED);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_a->GetState(), Task::COMPLETED);
  EXPECT_EQ(shared_b->GetState(), Task::COMPLETED);
}

TEST_F(TaskTest, RunTaskWithTwoDependency) {
  /*         c \
   *  a -->  b --> d
   */
  auto a = make_unique<Task>();
  auto b = make_unique<Task>();
  auto c = make_unique<Task>();
  auto d = make_unique<Task>();
  MockCallback callback_a;
  a->SetWorkItem([&callback_a]() { callback_a.Run(); });
  MockCallback callback_b;
  b->SetWorkItem([&callback_b]() { callback_b.Run(); });
  MockCallback callback_c;
  c->SetWorkItem([&callback_c]() { callback_c.Run(); });
  MockCallback callback_d;
  d->SetWorkItem([&callback_d]() { callback_d.Run(); });
  EXPECT_CALL(callback_a, Run()).Times(1);
  EXPECT_CALL(callback_b, Run()).Times(1);
  EXPECT_CALL(callback_c, Run()).Times(1);
  EXPECT_CALL(callback_d, Run()).Times(1);
  auto shared_a = thread_pool()->Schedule(std::move(a)).lock();
  EXPECT_NE(shared_a, nullptr);
  b->AddDependency(shared_a);
  auto shared_b = thread_pool()->Schedule(std::move(b)).lock();
  EXPECT_NE(shared_b, nullptr);
  auto shared_c = thread_pool()->Schedule(std::move(c)).lock();
  EXPECT_NE(shared_c, nullptr);
  d->AddDependency(shared_b);
  d->AddDependency(shared_c);
  auto shared_d = thread_pool()->Schedule(std::move(d)).lock();
  EXPECT_NE(shared_d, nullptr);
  EXPECT_EQ(shared_b->GetState(), Task::DISPATCHED);
  EXPECT_EQ(shared_d->GetState(), Task::DISPATCHED);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_a->GetState(), Task::COMPLETED);
  EXPECT_EQ(shared_b->GetState(), Task::DEPENDENCIES_COMPLETED);
  EXPECT_EQ(shared_c->GetState(), Task::DEPENDENCIES_COMPLETED);
  thread_pool()->RunNext();
  thread_pool()->RunNext();
  EXPECT_EQ(shared_b->GetState(), Task::COMPLETED);
  EXPECT_EQ(shared_c->GetState(), Task::COMPLETED);
  EXPECT_EQ(shared_d->GetState(), Task::DEPENDENCIES_COMPLETED);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_d->GetState(), Task::COMPLETED);
}

TEST_F(TaskTest, RunWithCompletedDependency) {
  auto a = make_unique<Task>();
  MockCallback callback_a;
  a->SetWorkItem([&callback_a]() { callback_a.Run(); });
  auto shared_a = thread_pool()->Schedule(std::move(a)).lock();
  EXPECT_NE(shared_a, nullptr);
  EXPECT_EQ(shared_a->GetState(), Task::DEPENDENCIES_COMPLETED);
  EXPECT_CALL(callback_a, Run()).Times(1);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_a->GetState(), Task::COMPLETED);
  auto b = make_unique<Task>();
  MockCallback callback_b;
  b->SetWorkItem([&callback_b]() { callback_b.Run(); });
  b->AddDependency(shared_a);
  EXPECT_EQ(b->GetState(), Task::NEW);
  auto shared_b = thread_pool()->Schedule(std::move(b)).lock();
  EXPECT_NE(shared_b, nullptr);
  EXPECT_EQ(shared_b->GetState(), Task::DEPENDENCIES_COMPLETED);
  EXPECT_CALL(callback_b, Run()).Times(1);
  thread_pool()->RunNext();
  EXPECT_EQ(shared_b->GetState(), Task::COMPLETED);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
