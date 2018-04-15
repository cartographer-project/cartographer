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

class Task {
 public:
  using WorkItem = std::function<void()>;
  using TaskDispatcher = std::function<void(Task*)>;
  using TasksDispatchingWorkItem = std::function<void(TaskDispatcher)>;
  enum State { IDLE, DISPATCHED, RUNNING, COMPLETED };

  Task() {}
  Task(WorkItem work_item) : work_item_(work_item) {}

  State GetState() { return state_; }
  void AddDependency(Task* dependency);
  void Dispatch(ThreadPoolInterface* thread_pool);
  void AddDependentTask(Task* dependent_task);
  void OnDependenyCompleted();
  WorkItem ContructThreadPoolWorkItem();

 protected:

  WorkItem work_item_;
  ThreadPoolInterface* thread_pool_ = nullptr;
  State state_ = IDLE;
  unsigned int ref_count_ = 0;
  std::set<Task*> dependent_tasks_;

  Mutex mutex_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_
