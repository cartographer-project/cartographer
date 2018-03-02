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

#include "cartographer_grpc/internal/framework/completion_queue_thread.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

CompletionQueueThread::CompletionQueueThread(
    std::unique_ptr<::grpc::ServerCompletionQueue> completion_queue)
    : completion_queue_(std::move(completion_queue)) {}

::grpc::ServerCompletionQueue* CompletionQueueThread::completion_queue() {
  return completion_queue_.get();
}

void CompletionQueueThread::Start(CompletionQueueRunner runner) {
  CHECK(!worker_thread_);
  worker_thread_ = cartographer::common::make_unique<std::thread>(
      [this, runner]() { runner(this->completion_queue_.get()); });
}

void CompletionQueueThread::Shutdown() {
  LOG(INFO) << "Shutting down completion queue " << completion_queue_.get();
  completion_queue_->Shutdown();
  worker_thread_->join();
}

}  // namespace framework
}  // namespace cartographer_grpc
