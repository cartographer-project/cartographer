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

namespace cartographer {
namespace common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

Task::State Task::GetState() {
  MutexLocker locker(&mutex_);
  return state_;
}

void Task::SetWorkItem(const WorkItem& work_item) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, NEW);
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  if (shared_dependency) {
    shared_dependency->AddDependentTask(this);
  }
}

void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  state_ = DISPATCHED;
  thread_pool_to_notify_ = thread_pool;
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

void Task::AddDependentTask(Task* dependent_task) {
  MutexLocker locker(&mutex_);
  if (state_ == COMPLETED) {
    dependent_task->OnDependenyCompleted();
    return;
  }
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

void Task::OnDependenyCompleted() {
  MutexLocker locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

void Task::Execute() {
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  MutexLocker locker(&mutex_);
  state_ = COMPLETED;
  for (Task* dependent_task : dependent_tasks_) {
    dependent_task->OnDependenyCompleted();
  }
}

}  // namespace common
}  // namespace cartographer
