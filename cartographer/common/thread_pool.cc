/*
 * Copyright 2016 The Cartographer Authors
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

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/task.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(task_queue_.size(), 0);
    CHECK_EQ(tasks_not_ready_.size(), 0);
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

void ThreadPool::NotifyReady(Task* task) {
  MutexLocker locker(&mutex_);
  CHECK(running_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);
  tasks_not_ready_.erase(it);
}

#if 0
void ThreadPool::Schedule(const std::function<void()>& work_item) {
  MutexLocker locker(&mutex_);
  CHECK(running_);
  auto task = make_unique<Task>();
  task->SetWorkItem(work_item);
  task_queue_.push_back(std::move(task));
}
#endif

std::shared_ptr<Task> ThreadPool::ScheduleWhenReady(
    std::shared_ptr<Task> task) {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), task));
    CHECK(insert_result.second) << "ScheduleWhenReady called twice";
  }
  task->NotifyWhenReady(this);
  return task;
}

void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::shared_ptr<Task> task;
    {
      MutexLocker locker(&mutex_);
      locker.Await([this]() REQUIRES(mutex_) {
        return !task_queue_.empty() || !running_;
      });
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::READY);
    task->Execute();
  }
}

void ThreadPool::WaitForAllForTesting() {
  bool finished = false;
  while (!finished) {
    common::MutexLocker locker(&mutex_);
    finished = locker.AwaitWithTimeout(
        [this]() REQUIRES(mutex_) {
          return (tasks_not_ready_.empty() && task_queue_.empty());
        },
        common::FromSeconds(0.1));
    LOG(INFO) << "tasks_not_ready_.size(): " << tasks_not_ready_.size();
    LOG(INFO) << "task_queue_.size(): " << task_queue_.size();
  }
  CHECK(tasks_not_ready_.empty() && task_queue_.empty());
  CHECK(running_);
}

}  // namespace common
}  // namespace cartographer
