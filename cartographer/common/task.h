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

 public:
  Task() {}
  Task(WorkItem work_item) : work_item_(work_item) {}

  void AddDependency(Task* dependency) {
    {
      MutexLocker locker(&mutex_);
      CHECK_EQ(state_, IDLE);
      ++ref_count_;
    }
    dependency->AddDependentTask(this);
  }

  void Dispatch(ThreadPool* thread_pool) {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, IDLE);
    state_ = DISPATCHED;
    thread_pool_ = thread_pool;
    if (ref_count_ == 0) {
      CHECK(thread_pool_);
      thread_pool_->Schedule(ContructThreadPoolWorkItem());
    }
  }

  void AddDependentTask(Task* dependent_task) {
	MutexLocker locker(&mutex_);
	if (state_ == COMPLETED) {
	  dependent_task->OnDependenyCompleted();
	  return;
	}
	dependent_tasks_.insert(dependent_task);
  }

  virtual void OnDependenyCompleted() {
	MutexLocker locker(&mutex_);
	--ref_count_;
	if (ref_count_ == 0 && state_ == DISPATCHED) {
	  CHECK(thread_pool_);
	  thread_pool_->Schedule(ContructThreadPoolWorkItem());
	}
  }

  virtual WorkItem ContructThreadPoolWorkItem() {
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

 protected:
  enum State { IDLE, DISPATCHED, RUNNING, COMPLETED };

  WorkItem work_item_;
  ThreadPool* thread_pool_ = nullptr;
  State state_ = IDLE;
  unsigned int ref_count_ = 0;
  std::set<Task*> dependent_tasks_;

  Mutex mutex_;
};

class TasksSchedulingTask : public Task {
 public:
  TasksSchedulingTask(TasksDispatchingWorkItem task_dispatching_work_item) : task_dispatching_work_item_(task_dispatching_work_item) {}

 protected:
  TaskDispatcher ConstructTaskDispatcher() {
	return [this](Task* dependency) {
		{
		  MutexLocker locker(&mutex_);
		  CHECK_EQ(state_, RUNNING);
		  ++ref_count_;
		}
		dependency->AddDependentTask(this);
		dependency->Dispatch(thread_pool_);
	};
  }

  void OnDependenyCompleted() override {
	MutexLocker locker(&mutex_);
	CHECK(state_ != IDLE);
	CHECK(state_ != COMPLETED);
	--ref_count_;
	if (ref_count_ == 0 && state_ == DISPATCHED) {
	  CHECK(thread_pool_);
	  thread_pool_->Schedule(ContructThreadPoolWorkItem());
	} else if (ref_count_ == 0 && state_ == RUNNING) {
	  state_ = COMPLETED;
	  for (Task* dependent_task : dependent_tasks_) {
	    dependent_task->OnDependenyCompleted();
	  }
	}
  }

  WorkItem ContructThreadPoolWorkItem() override {
	return [this]() {
		{
		  MutexLocker locker(&mutex_);
		  state_ = RUNNING;

		  // Up ref count by one to ensure this task does
		  // not complete prematurely while dispatching
		  // work items.
          ++ref_count_;
		  CHECK_EQ(ref_count_, 1);
	    }
		task_dispatching_work_item_(ConstructTaskDispatcher());
        OnDependenyCompleted();
	};
  }

  TasksDispatchingWorkItem task_dispatching_work_item_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_
