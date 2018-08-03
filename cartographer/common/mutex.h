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

#ifndef CARTOGRAPHER_COMMON_MUTEX_H_
#define CARTOGRAPHER_COMMON_MUTEX_H_

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define EXCLUDES(...) THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

// TODO(CodeArno): Replace references in code to absl::Mutex directly.
using Mutex = absl::Mutex;

// A RAII class that acquires a mutex in its constructor, and
// releases it in its destructor. It also implements waiting functionality on
// conditions that get checked whenever the mutex is released.
// TODO(CodeArno): Replace MutexLocker instantiations in the codebase with
// absl::MutexLock.
class SCOPED_LOCKABLE MutexLocker {
 public:
  MutexLocker(Mutex* mutex) EXCLUSIVE_LOCK_FUNCTION(mutex)
      : mutex_(mutex), lock_(mutex) {}

  ~MutexLocker() UNLOCK_FUNCTION() {}

  template <typename Predicate>
  void Await(Predicate predicate) REQUIRES(this) {
    mutex_->Await(absl::Condition(&predicate));
  }

  template <typename Predicate>
  bool AwaitWithTimeout(Predicate predicate, common::Duration timeout)
      REQUIRES(this) {
    return mutex_->AwaitWithTimeout(absl::Condition(&predicate),
                                    absl::FromChrono(timeout));
  }

 private:
  absl::Mutex* mutex_;
  absl::MutexLock lock_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MUTEX_H_
