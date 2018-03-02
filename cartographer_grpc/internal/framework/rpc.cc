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

#include "cartographer_grpc/internal/framework/rpc.h"
#include "cartographer_grpc/internal/framework/service.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer_grpc {
namespace framework {
namespace {

// Finishes the gRPC for non-streaming response RPCs, i.e. NORMAL_RPC and
// CLIENT_STREAMING. If no 'msg' is passed, we signal an error to the client as
// the server is not honoring the gRPC call signature.
template <typename ReaderWriter>
void SendUnaryFinish(ReaderWriter* reader_writer, ::grpc::Status status,
                     const google::protobuf::Message* msg,
                     Rpc::EventBase* rpc_event) {
  if (msg) {
    reader_writer->Finish(*msg, status, rpc_event);
  } else {
    reader_writer->FinishWithError(status, rpc_event);
  }
}

}  // namespace

void Rpc::CompletionQueueRpcEvent::Handle() {
  pending = false;
  rpc_ptr->service()->HandleEvent(event, rpc_ptr, ok);
}

void Rpc::InternalRpcEvent::Handle() {
  if (auto rpc_shared = rpc.lock()) {
    rpc_shared->service()->HandleEvent(event, rpc_shared.get(), true);
  } else {
    LOG(WARNING) << "Ignoring stale event.";
  }
}

Rpc::Rpc(int method_index,
         ::grpc::ServerCompletionQueue* server_completion_queue,
         EventQueue* event_queue, ExecutionContext* execution_context,
         const RpcHandlerInfo& rpc_handler_info, Service* service,
         WeakPtrFactory weak_ptr_factory)
    : method_index_(method_index),
      server_completion_queue_(server_completion_queue),
      event_queue_(event_queue),
      execution_context_(execution_context),
      rpc_handler_info_(rpc_handler_info),
      service_(service),
      weak_ptr_factory_(weak_ptr_factory),
      new_connection_event_(Event::NEW_CONNECTION, this),
      read_event_(Event::READ, this),
      write_event_(Event::WRITE, this),
      finish_event_(Event::FINISH, this),
      done_event_(Event::DONE, this),
      handler_(rpc_handler_info_.rpc_handler_factory(this, execution_context)) {
  InitializeReadersAndWriters(rpc_handler_info_.rpc_type);

  // Initialize the prototypical request and response messages.
  request_.reset(::google::protobuf::MessageFactory::generated_factory()
                     ->GetPrototype(rpc_handler_info_.request_descriptor)
                     ->New());
  response_.reset(::google::protobuf::MessageFactory::generated_factory()
                      ->GetPrototype(rpc_handler_info_.response_descriptor)
                      ->New());
}

std::unique_ptr<Rpc> Rpc::Clone() {
  return cartographer::common::make_unique<Rpc>(
      method_index_, server_completion_queue_, event_queue_, execution_context_,
      rpc_handler_info_, service_, weak_ptr_factory_);
}

void Rpc::OnRequest() { handler_->OnRequestInternal(request_.get()); }

void Rpc::OnReadsDone() { handler_->OnReadsDone(); }

void Rpc::OnFinish() { handler_->OnFinish(); }

void Rpc::RequestNextMethodInvocation() {
  // Ask gRPC to notify us when the connection terminates.
  SetRpcEventState(Event::DONE, true);
  // TODO(gaschler): Asan reports direct leak of this new from both calls
  // StartServing and HandleNewConnection.
  server_context_.AsyncNotifyWhenDone(GetRpcEvent(Event::DONE));

  // Make sure after terminating the connection, gRPC notifies us with this
  // event.
  SetRpcEventState(Event::NEW_CONNECTION, true);
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      service_->RequestAsyncBidiStreaming(
          method_index_, &server_context_, streaming_interface(),
          server_completion_queue_, server_completion_queue_,
          GetRpcEvent(Event::NEW_CONNECTION));
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      service_->RequestAsyncClientStreaming(
          method_index_, &server_context_, streaming_interface(),
          server_completion_queue_, server_completion_queue_,
          GetRpcEvent(Event::NEW_CONNECTION));
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      service_->RequestAsyncUnary(
          method_index_, &server_context_, request_.get(),
          streaming_interface(), server_completion_queue_,
          server_completion_queue_, GetRpcEvent(Event::NEW_CONNECTION));
      break;
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      service_->RequestAsyncServerStreaming(
          method_index_, &server_context_, request_.get(),
          streaming_interface(), server_completion_queue_,
          server_completion_queue_, GetRpcEvent(Event::NEW_CONNECTION));
      break;
  }
}

void Rpc::RequestStreamingReadIfNeeded() {
  // For request-streaming RPCs ask the client to start sending requests.
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      SetRpcEventState(Event::READ, true);
      async_reader_interface()->Read(request_.get(), GetRpcEvent(Event::READ));
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      // For NORMAL_RPC and SERVER_STREAMING we don't need to queue an event,
      // since gRPC automatically issues a READ request and places the request
      // into the 'Message' we provided to 'RequestAsyncUnary' above.
      OnRequest();
      OnReadsDone();
      break;
  }
}

void Rpc::Write(std::unique_ptr<::google::protobuf::Message> message) {
  EnqueueMessage(SendItem{std::move(message), ::grpc::Status::OK});
  event_queue_->Push(UniqueEventPtr(
      new InternalRpcEvent(Event::WRITE_NEEDED, weak_ptr_factory_(this))));
}

void Rpc::Finish(::grpc::Status status) {
  EnqueueMessage(SendItem{nullptr /* message */, status});
  event_queue_->Push(UniqueEventPtr(
      new InternalRpcEvent(Event::WRITE_NEEDED, weak_ptr_factory_(this))));
}

void Rpc::HandleSendQueue() {
  SendItem send_item;
  {
    cartographer::common::MutexLocker locker(&send_queue_lock_);
    if (send_queue_.empty() || IsRpcEventPending(Event::WRITE) ||
        IsRpcEventPending(Event::FINISH)) {
      return;
    }

    send_item = std::move(send_queue_.front());
    send_queue_.pop();
  }
  if (!send_item.msg ||
      rpc_handler_info_.rpc_type == ::grpc::internal::RpcMethod::NORMAL_RPC ||
      rpc_handler_info_.rpc_type ==
          ::grpc::internal::RpcMethod::CLIENT_STREAMING) {
    PerformFinish(std::move(send_item.msg), send_item.status);
    return;
  }
  PerformWrite(std::move(send_item.msg), send_item.status);
}

::grpc::internal::ServerAsyncStreamingInterface* Rpc::streaming_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      return server_async_response_writer_.get();
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      return server_async_writer_.get();
  }
  LOG(FATAL) << "Never reached.";
}

::grpc::internal::AsyncReaderInterface<::google::protobuf::Message>*
Rpc::async_reader_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      return server_async_reader_.get();
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      LOG(FATAL) << "For NORMAL_RPC no streaming reader interface exists.";
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      LOG(FATAL)
          << "For SERVER_STREAMING no streaming reader interface exists.";
  }
  LOG(FATAL) << "Never reached.";
}

::grpc::internal::AsyncWriterInterface<::google::protobuf::Message>*
Rpc::async_writer_interface() {
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      return server_async_reader_writer_.get();
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      LOG(FATAL) << "For NORMAL_RPC and CLIENT_STREAMING no streaming writer "
                    "interface exists.";
      break;
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      return server_async_writer_.get();
  }
  LOG(FATAL) << "Never reached.";
}

Rpc::CompletionQueueRpcEvent* Rpc::GetRpcEvent(Event event) {
  switch (event) {
    case Event::NEW_CONNECTION:
      return &new_connection_event_;
    case Event::READ:
      return &read_event_;
    case Event::WRITE_NEEDED:
      LOG(FATAL) << "Rpc does not store Event::WRITE_NEEDED.";
      break;
    case Event::WRITE:
      return &write_event_;
    case Event::FINISH:
      return &finish_event_;
    case Event::DONE:
      return &done_event_;
  }
  LOG(FATAL) << "Never reached.";
}

bool* Rpc::GetRpcEventState(Event event) {
  return &GetRpcEvent(event)->pending;
}

void Rpc::EnqueueMessage(SendItem&& send_item) {
  cartographer::common::MutexLocker locker(&send_queue_lock_);
  send_queue_.emplace(std::move(send_item));
}

void Rpc::PerformFinish(std::unique_ptr<::google::protobuf::Message> message,
                        ::grpc::Status status) {
  SetRpcEventState(Event::FINISH, true);
  switch (rpc_handler_info_.rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      CHECK(!message);
      server_async_reader_writer_->Finish(status, GetRpcEvent(Event::FINISH));
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      response_ = std::move(message);
      SendUnaryFinish(server_async_reader_.get(), status, response_.get(),
                      GetRpcEvent(Event::FINISH));
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      response_ = std::move(message);
      SendUnaryFinish(server_async_response_writer_.get(), status,
                      response_.get(), GetRpcEvent(Event::FINISH));
      break;
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      CHECK(!message);
      server_async_writer_->Finish(status, GetRpcEvent(Event::FINISH));
      break;
  }
}

void Rpc::PerformWrite(std::unique_ptr<::google::protobuf::Message> message,
                       ::grpc::Status status) {
  CHECK(message) << "PerformWrite must be called with a non-null message";
  CHECK_NE(rpc_handler_info_.rpc_type, ::grpc::internal::RpcMethod::NORMAL_RPC);
  CHECK_NE(rpc_handler_info_.rpc_type,
           ::grpc::internal::RpcMethod::CLIENT_STREAMING);
  SetRpcEventState(Event::WRITE, true);
  response_ = std::move(message);
  async_writer_interface()->Write(*response_, GetRpcEvent(Event::WRITE));
}

void Rpc::SetRpcEventState(Event event, bool pending) {
  // TODO(gaschler): Since the only usage is setting this true at creation,
  // consider removing this method.
  *GetRpcEventState(event) = pending;
}

bool Rpc::IsRpcEventPending(Event event) { return *GetRpcEventState(event); }

bool Rpc::IsAnyEventPending() {
  return IsRpcEventPending(Rpc::Event::DONE) ||
         IsRpcEventPending(Rpc::Event::READ) ||
         IsRpcEventPending(Rpc::Event::WRITE) ||
         IsRpcEventPending(Rpc::Event::FINISH);
}

std::weak_ptr<Rpc> Rpc::GetWeakPtr() { return weak_ptr_factory_(this); }

ActiveRpcs::ActiveRpcs() : lock_() {}

void Rpc::InitializeReadersAndWriters(
    ::grpc::internal::RpcMethod::RpcType rpc_type) {
  switch (rpc_type) {
    case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      server_async_reader_writer_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReaderWriter<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
      server_async_reader_ =
          cartographer::common::make_unique<::grpc::ServerAsyncReader<
              google::protobuf::Message, google::protobuf::Message>>(
              &server_context_);
      break;
    case ::grpc::internal::RpcMethod::NORMAL_RPC:
      server_async_response_writer_ = cartographer::common::make_unique<
          ::grpc::ServerAsyncResponseWriter<google::protobuf::Message>>(
          &server_context_);
      break;
    case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      server_async_writer_ = cartographer::common::make_unique<
          ::grpc::ServerAsyncWriter<google::protobuf::Message>>(
          &server_context_);
      break;
  }
}

ActiveRpcs::~ActiveRpcs() {
  cartographer::common::MutexLocker locker(&lock_);
  if (!rpcs_.empty()) {
    LOG(FATAL) << "RPCs still in flight!";
  }
}

std::shared_ptr<Rpc> ActiveRpcs::Add(std::unique_ptr<Rpc> rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  std::shared_ptr<Rpc> shared_ptr_rpc = std::move(rpc);
  const auto result = rpcs_.emplace(shared_ptr_rpc.get(), shared_ptr_rpc);
  CHECK(result.second) << "RPC already active.";
  return shared_ptr_rpc;
}

bool ActiveRpcs::Remove(Rpc* rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  auto it = rpcs_.find(rpc);
  if (it != rpcs_.end()) {
    rpcs_.erase(it);
    return true;
  }
  return false;
}

Rpc::WeakPtrFactory ActiveRpcs::GetWeakPtrFactory() {
  return [this](Rpc* rpc) { return GetWeakPtr(rpc); };
}

std::weak_ptr<Rpc> ActiveRpcs::GetWeakPtr(Rpc* rpc) {
  cartographer::common::MutexLocker locker(&lock_);
  auto it = rpcs_.find(rpc);
  CHECK(it != rpcs_.end());
  return it->second;
}

}  // namespace framework
}  // namespace cartographer_grpc
