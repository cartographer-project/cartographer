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

#include <list>

#include "cartographer/common/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace cartographer {
namespace common {

class Task {
 public:
  using WorkItem = std::function<void()>;
  using Scheduler = std::function<void(Task*)>;
  using SpawningWorkItem = std::function<void(Scheduler)>;

 public:
  Task(ThreadPool* thread_pool) : thread_pool_(thread_pool) {}

  void Schedule() { Schedule({}); }

  void Schedule(std::list<Task*> dependencies) {
    // Don't allow to schedule the task twice.
    CHECK(!done_);

    {  // ref_count_mutex_
      Mutex::Locker ref_count_locker(&ref_count_mutex_);
      CHECK(ref_count_ == 0);
      ref_count_ = dependencies.size();
      for (Task* t : dependencies) {
        if (!t->AddDependent(this)) {
          --ref_count_;
        }
      }
      if (ref_count_ == 0) {
        // All dependencies already fulfilled, so Task schedules itself.
        ScheduleOnThreadPool();
      }
    }  // !ref_count_mutex_
  }

  void ScheduleOnThreadPool() {
    thread_pool_->Schedule(ConstructThreadPoolWorkItem());
  }

  virtual WorkItem ConstructThreadPoolWorkItem() = 0;

  void Notify() {
    Mutex::Locker ref_count_locker(&ref_count_mutex_);
    CHECK_GE(ref_count_, 0);
    --ref_count_;
    if (ref_count_ == 0) {
      ScheduleOnThreadPool();
    }
  }

  bool AddDependent(Task* dependent) {
    {  // done_mutex_
      Mutex::Locker done_locker(&done_mutex_);
      if (done_) {
        // Task already finished and does not accept dependents anymore.
        return false;
      } else {
        {  // dependents_mutex_
          Mutex::Locker dependents_locker(&dependents_mutex_);
          dependents_.push_back(dependent);
          return true;
        }  // ! dependents_mutex_
      }
    }  // ! done_mutex_
  }

 protected:
  ThreadPool* thread_pool_;

  Mutex ref_count_mutex_;
  unsigned int ref_count_ = 0;

  Mutex dependents_mutex_;
  std::list<Task*> dependents_ = {};

  Mutex done_mutex_;
  bool done_ = false;
};

/**
 * SimpleTask a(..., p);
 * SimpleTask b(..., p);
 *
 * a.Schedule({});
 * b.Schedule({&a});
 */
class SimpleTask : public Task {
 public:
  SimpleTask(const WorkItem work_item, ThreadPool* thread_pool)
      : Task(thread_pool), work_item_(work_item) {}

  WorkItem ConstructThreadPoolWorkItem() override {
    return [this] {
      // Execute the work item.
      work_item_();

      {  // done_mutex_
        Mutex::Locker done_locker(&done_mutex_);
        done_ = true;
      }  // ! done_mutex_

      {  // dependents_mutex_
        Mutex::Locker dependents_locker(&dependents_mutex_);
        for (Task* t : dependents_) {
          t->Notify();
        }
      }  // ! dependents_mutex_
    };
  }

 private:
  const WorkItem work_item_;
};

/**
 * A task that spawn other tasks and only completes when all of its children
 * completed.
 *
 * SpawnTask s([](Task::Scheduler scheduler) {
 *
 *     }, p);
 * s.Schedule({});
 */
class SpawnTask : public Task {
 public:
  SpawnTask(SpawningWorkItem work_item, ThreadPool* thread_pool)
      : Task(thread_pool), work_item_(work_item) {}

  WorkItem ConstructThreadPoolWorkItem() override {
    return [this] {
      work_item_([this](Task* task) {

      });
    };
  }

 private:
  const SpawningWorkItem work_item_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_