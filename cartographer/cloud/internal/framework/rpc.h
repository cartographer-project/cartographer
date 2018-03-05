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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_H
#define CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_H

#include <memory>
#include <queue>
#include <unordered_set>

#include "cartographer/cloud/internal/framework/execution_context.h"
#include "cartographer/cloud/internal/framework/rpc_handler_interface.h"
#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/mutex.h"
#include "google/protobuf/message.h"
#include "grpc++/grpc++.h"
#include "grpc++/impl/codegen/async_stream.h"
#include "grpc++/impl/codegen/async_unary_call.h"
#include "grpc++/impl/codegen/proto_utils.h"
#include "grpc++/impl/codegen/service_type.h"

namespace cartographer {
namespace cloud {
namespace framework {

class Service;
// TODO(cschuet): Add a unittest that tests the logic of this class.
class Rpc {
 public:
  using WeakPtrFactory = std::function<std::weak_ptr<Rpc>(Rpc*)>;
  enum class Event {
    NEW_CONNECTION = 0,
    READ,
    WRITE_NEEDED,
    WRITE,
    FINISH,
    DONE
  };

  struct EventBase {
    explicit EventBase(Event event) : event(event) {}
    virtual ~EventBase(){};
    virtual void Handle() = 0;

    const Event event;
  };

  class EventDeleter {
   public:
    enum Action { DELETE = 0, DO_NOT_DELETE };

    // The default action 'DELETE' is used implicitly, for instance for a
    // new UniqueEventPtr or a UniqueEventPtr that is created by
    // 'return nullptr'.
    EventDeleter() : action_(DELETE) {}
    explicit EventDeleter(Action action) : action_(action) {}
    void operator()(EventBase* e) {
      if (e != nullptr && action_ == DELETE) {
        delete e;
      }
    }

   private:
    Action action_;
  };

  using UniqueEventPtr = std::unique_ptr<EventBase, EventDeleter>;
  using EventQueue = common::BlockingQueue<UniqueEventPtr>;

  // Flows through gRPC's CompletionQueue and then our EventQueue.
  struct CompletionQueueRpcEvent : public EventBase {
    CompletionQueueRpcEvent(Event event, Rpc* rpc)
        : EventBase(event), rpc_ptr(rpc), ok(false), pending(false) {}
    void PushToEventQueue() {
      rpc_ptr->event_queue()->Push(
          UniqueEventPtr(this, EventDeleter(EventDeleter::DO_NOT_DELETE)));
    }
    void Handle() override;

    Rpc* rpc_ptr;
    bool ok;
    bool pending;
  };

  // Flows only through our EventQueue.
  struct InternalRpcEvent : public EventBase {
    InternalRpcEvent(Event event, std::weak_ptr<Rpc> rpc)
        : EventBase(event), rpc(rpc) {}
    void Handle() override;

    std::weak_ptr<Rpc> rpc;
  };

  Rpc(int method_index, ::grpc::ServerCompletionQueue* server_completion_queue,
      EventQueue* event_queue, ExecutionContext* execution_context,
      const RpcHandlerInfo& rpc_handler_info, Service* service,
      WeakPtrFactory weak_ptr_factory);
  std::unique_ptr<Rpc> Clone();
  void OnRequest();
  void OnReadsDone();
  void OnFinish();
  void RequestNextMethodInvocation();
  void RequestStreamingReadIfNeeded();
  void HandleSendQueue();
  void Write(std::unique_ptr<::google::protobuf::Message> message);
  void Finish(::grpc::Status status);
  Service* service() { return service_; }
  bool IsRpcEventPending(Event event);
  bool IsAnyEventPending();
  void SetEventQueue(EventQueue* event_queue) { event_queue_ = event_queue; }
  EventQueue* event_queue() { return event_queue_; }
  std::weak_ptr<Rpc> GetWeakPtr();

 private:
  struct SendItem {
    std::unique_ptr<google::protobuf::Message> msg;
    ::grpc::Status status;
  };

  Rpc(const Rpc&) = delete;
  Rpc& operator=(const Rpc&) = delete;
  void InitializeReadersAndWriters(
      ::grpc::internal::RpcMethod::RpcType rpc_type);
  CompletionQueueRpcEvent* GetRpcEvent(Event event);
  bool* GetRpcEventState(Event event);
  void SetRpcEventState(Event event, bool pending);
  void EnqueueMessage(SendItem&& send_item);
  void PerformFinish(std::unique_ptr<::google::protobuf::Message> message,
                     ::grpc::Status status);
  void PerformWrite(std::unique_ptr<::google::protobuf::Message> message,
                    ::grpc::Status status);

  ::grpc::internal::AsyncReaderInterface<::google::protobuf::Message>*
  async_reader_interface();
  ::grpc::internal::AsyncWriterInterface<::google::protobuf::Message>*
  async_writer_interface();

  ::grpc::internal::ServerAsyncStreamingInterface* streaming_interface();

  int method_index_;
  ::grpc::ServerCompletionQueue* server_completion_queue_;
  EventQueue* event_queue_;
  ExecutionContext* execution_context_;
  RpcHandlerInfo rpc_handler_info_;
  Service* service_;
  WeakPtrFactory weak_ptr_factory_;
  ::grpc::ServerContext server_context_;

  CompletionQueueRpcEvent new_connection_event_;
  CompletionQueueRpcEvent read_event_;
  CompletionQueueRpcEvent write_event_;
  CompletionQueueRpcEvent finish_event_;
  CompletionQueueRpcEvent done_event_;

  std::unique_ptr<google::protobuf::Message> request_;
  std::unique_ptr<google::protobuf::Message> response_;

  std::unique_ptr<RpcHandlerInterface> handler_;

  std::unique_ptr<::grpc::ServerAsyncResponseWriter<google::protobuf::Message>>
      server_async_response_writer_;
  std::unique_ptr<::grpc::ServerAsyncReader<google::protobuf::Message,
                                            google::protobuf::Message>>
      server_async_reader_;
  std::unique_ptr<::grpc::ServerAsyncReaderWriter<google::protobuf::Message,
                                                  google::protobuf::Message>>
      server_async_reader_writer_;
  std::unique_ptr<::grpc::ServerAsyncWriter<google::protobuf::Message>>
      server_async_writer_;

  common::Mutex send_queue_lock_;
  std::queue<SendItem> send_queue_;
};

using EventQueue = Rpc::EventQueue;

// This class keeps track of all in-flight RPCs for a 'Service'. Make sure that
// all RPCs have been terminated and removed from this object before it goes out
// of scope.
class ActiveRpcs {
 public:
  ActiveRpcs();
  ~ActiveRpcs() EXCLUDES(lock_);

  std::shared_ptr<Rpc> Add(std::unique_ptr<Rpc> rpc) EXCLUDES(lock_);
  bool Remove(Rpc* rpc) EXCLUDES(lock_);
  Rpc::WeakPtrFactory GetWeakPtrFactory();

 private:
  std::weak_ptr<Rpc> GetWeakPtr(Rpc* rpc);

  common::Mutex lock_;
  std::map<Rpc*, std::shared_ptr<Rpc>> rpcs_;
};

}  // namespace framework
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_FRAMEWORK_RPC_H
