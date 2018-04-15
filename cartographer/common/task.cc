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


void Task::AddDependency(Task* dependency) {
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, IDLE);
    ++ref_count_;
  }
  dependency->AddDependentTask(this);
}

void Task::Dispatch(ThreadPoolInterface* thread_pool) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, IDLE);
  state_ = DISPATCHED;
  thread_pool_ = thread_pool;
  if (ref_count_ == 0) {
    CHECK(thread_pool_);
    thread_pool_->Schedule(ContructThreadPoolWorkItem());
  }
}

void Task::AddDependentTask(Task* dependent_task) {
	MutexLocker locker(&mutex_);
	if (state_ == COMPLETED) {
	  dependent_task->OnDependenyCompleted();
	  return;
	}
	dependent_tasks_.insert(dependent_task);
}

void Task::OnDependenyCompleted() {
	MutexLocker locker(&mutex_);
	--ref_count_;
	if (ref_count_ == 0 && state_ == DISPATCHED) {
	  CHECK(thread_pool_);
	  thread_pool_->Schedule(ContructThreadPoolWorkItem());
	}
}

Task::WorkItem Task::ContructThreadPoolWorkItem() {
  return [this]() {
    {
      MutexLocker locker(&mutex_);
  	state_ = RUNNING;
    }

    // Execute the work item.
    if(work_item_) {
  	  work_item_();
    }

    MutexLocker locker(&mutex_);
    state_ = COMPLETED;
    for (Task* dependent_task : dependent_tasks_) {
  	dependent_task->OnDependenyCompleted();
    }
  };
}

}  // namespace common
}  // namespace cartographer
