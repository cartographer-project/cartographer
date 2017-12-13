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

#include "cartographer_grpc/framework/event_queue_thread.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {

EventQueueThread::EventQueueThread() {
  event_queue_ = cartographer::common::make_unique<EventQueue>();
}

EventQueue* EventQueueThread::event_queue() {
  return event_queue_.get();
}

void EventQueueThread::Start(EventQueueRunner runner) {
  CHECK(!worker_thread_);
  worker_thread_ = cartographer::common::make_unique<std::thread>(
      [this, runner]() { runner(this->event_queue_.get()); });
}

void EventQueueThread::Shutdown() {
  LOG(INFO) << "Shutting down event queue " << event_queue_.get();
  worker_thread_->join();
}

}  // namespace framework
}  // namespace cartographer_grpc
