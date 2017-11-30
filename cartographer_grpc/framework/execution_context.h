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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_EXECUTION_CONTEXT_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_EXECUTION_CONTEXT_H

#include "cartographer/common/mutex.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

// Implementations of this class allow RPC handlers to share state among one
// another. Using Server::SetExecutionContext(...) a server-wide
// 'ExecutionContext' can be specified. This 'ExecutionContext' can be retrieved
// by all implementations of 'RpcHandler' by calling
// 'RpcHandler::GetContext<MyContext>()'.
class ExecutionContext {
 public:
  // This non-movable, non-copyable class is used to broker access from various
  // RPC handlers to the shared 'ExecutionContext'. Handles automatically lock
  // the context they point to.
  template <typename ContextType>
  class Synchronized {
   public:
    ContextType* operator->() {
      return static_cast<ContextType*>(execution_context_);
    }
    Synchronized(cartographer::common::Mutex* lock,
                 ExecutionContext* execution_context)
        : locker_(lock), execution_context_(execution_context) {}
    Synchronized(const Synchronized&) = delete;
    Synchronized(Synchronized&&) = delete;

   private:
    cartographer::common::MutexLocker locker_;
    ExecutionContext* execution_context_;
  };
  cartographer::common::Mutex* lock() { return &lock_; }

 private:
  cartographer::common::Mutex lock_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_EXECUTION_CONTEXT_H
