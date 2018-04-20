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

#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

class Task;

// General interface for thread pool implementations.
class ThreadPoolInterface {
 public:
  ThreadPoolInterface() {}
  virtual ~ThreadPoolInterface() {}
  virtual void NotifyReady(Task* task) = 0;
  // TODO(gaschler): Replace by ScheduleWhenReady.
  virtual void Schedule(const std::function<void()>& work_item) = 0;
  virtual std::shared_ptr<Task> ScheduleWhenReady(
      std::shared_ptr<Task> task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void NotifyReady(Task* task) EXCLUDES(mutex_) override;

  // TODO(gaschler): Remove all uses.
  void Schedule(const std::function<void()>& work_item) override;

  // TODO: Make this stricter.
  // If the returned smart pointer is expired, 'task' has certainly completed,
  // so it no longer needs be added as a dependency.
  std::shared_ptr<Task> ScheduleWhenReady(std::shared_ptr<Task> task)
      EXCLUDES(mutex_);

 private:
  void DoWork();

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  // TODO(gaschler): Make this a shared pointer at let Schedule return a weak
  // one.
  std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_);

  std::map<Task*, std::shared_ptr<Task>> tasks_not_ready_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_
