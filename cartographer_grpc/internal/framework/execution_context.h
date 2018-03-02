/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_EXECUTION_CONTEXT_H
#define CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_EXECUTION_CONTEXT_H

#include "cartographer/common/mutex.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {
namespace framework {

// Implementations of this class allow RPC handlers to share state among one
// another. Using Server::SetExecutionContext(...) a server-wide
// 'ExecutionContext' can be specified. This 'ExecutionContext' can be retrieved
// by all implementations of 'RpcHandler' by calling
// 'RpcHandler::GetContext<MyContext>()'.
class ExecutionContext {
 public:
  // Automatically locks an ExecutionContext for shared use by RPC handlers.
  // This non-movable, non-copyable class is used to broker access from various
  // RPC handlers to the shared 'ExecutionContext'.
  template <typename ContextType>
  class Synchronized {
   public:
    ContextType* operator->() {
      return static_cast<ContextType*>(execution_context_);
    }
    Synchronized(common::Mutex* lock, ExecutionContext* execution_context)
        : locker_(lock), execution_context_(execution_context) {}
    Synchronized(const Synchronized&) = delete;
    Synchronized(Synchronized&&) = delete;

   private:
    common::MutexLocker locker_;
    ExecutionContext* execution_context_;
  };
  ExecutionContext() = default;
  virtual ~ExecutionContext() = default;
  ExecutionContext(const ExecutionContext&) = delete;
  ExecutionContext& operator=(const ExecutionContext&) = delete;
  common::Mutex* lock() { return &lock_; }

 private:
  common::Mutex lock_;
};

}  // namespace framework
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_EXECUTION_CONTEXT_H
