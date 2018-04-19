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

#ifndef CARTOGRAPHER_COMMON_TASK_H_
#define CARTOGRAPHER_COMMON_TASK_H_

#include <set>

#include "cartographer/common/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace cartographer {
namespace common {

class ThreadPoolInterface;

class Task {
 public:
  using WorkItem = std::function<void()>;
  enum State { NEW, DISPATCHED, READY, RUNNING, COMPLETED };

  ~Task();

  State GetState() EXCLUDES(mutex_);

  // State must be 'NEW'.
  void SetWorkItem(const WorkItem& work_item) EXCLUDES(mutex_);
  // TODO(gaschler): Pass weak_ptr.

  // State must be 'NEW'.
  void AddDependency(Task* dependency) EXCLUDES(mutex_);

  // State must be 'NEW' and becomes 'DISPATCHED' or 'READY'.
  void NotifyWhenReady(ThreadPoolInterface* thread_pool) EXCLUDES(mutex_);

  // State must be 'READY' and becomes 'COMPLETED'.
  void Execute() EXCLUDES(mutex_);

 private:
  // Allowed all states.
  void AddDependentTask(Task* dependent_task);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become 'READY'.
  void OnDependenyCompleted();

  WorkItem work_item_ GUARDED_BY(mutex_);
  ThreadPoolInterface* thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
  State state_ GUARDED_BY(mutex_) = NEW;
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
  std::set<Task*> dependent_tasks_ GUARDED_BY(mutex_);

  Mutex mutex_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_
