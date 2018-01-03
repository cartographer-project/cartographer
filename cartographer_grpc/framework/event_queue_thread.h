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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_EVENT_QUEUE_THREAD_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_EVENT_QUEUE_THREAD_H

#include <memory>
#include <thread>

#include "cartographer/common/blocking_queue.h"
#include "cartographer_grpc/framework/rpc.h"

namespace cartographer_grpc {
namespace framework {

using EventQueue = cartographer::common::BlockingQueue<Rpc::RpcEvent*>;

class EventQueueThread {
 public:
  using EventQueueRunner = std::function<void(EventQueue*)>;

  EventQueueThread();

  EventQueue* event_queue();

  void Start(EventQueueRunner runner);
  void Shutdown();

 private:
  std::unique_ptr<EventQueue> event_queue_;
  std::unique_ptr<std::thread> thread_;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_EVENT_QUEUE_THREAD_H
