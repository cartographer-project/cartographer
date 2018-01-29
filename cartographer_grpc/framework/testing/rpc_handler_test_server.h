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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_
#define CARTOGRAPHER_GRPC_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_

#include <functional>
#include <string>

#include "cartographer/common/blocking_queue.h"
#include "cartographer_grpc/framework/server.h"
#include "cartographer_grpc/framework/testing/rpc_handler_wrapper.h"

namespace cartographer_grpc {
namespace framework {
namespace testing {

namespace {
const std::string kMethodName = "method";
const std::string kServiceName = "service";
const std::string kFullyQualifiedMethodName =
    "/" + kServiceName + "/" + kMethodName;
const std::string kServerAddress = "localhost:50051";
}  // namespace

template <class RpcHandlerType>
class RpcHandlerTestServer : public Server {
 public:
  RpcHandlerTestServer() : Server(Options{1, 1, kServerAddress}) {
    // Register the handler under test.
    this->AddService(kServiceName, {{kMethodName, GetRpcHandlerInfo()}});
    // Starts the server and instantiates the handler under test.
    this->Start();
    // Depending on the RPC type might already trigger a NEW_CONNECTION
    // event in the handler under test.
    InstantiateReadersAndWriters();
  }

  ~RpcHandlerTestServer() { this->Shutdown(); };

  // Parses a request message from the passed string and issues the
  // request against the handler, waits for the handler to complete
  // processing before returning.
  void SendWrite(const std::string &serialized_message) {
    auto message = cartographer::common::make_unique<
        typename RpcHandlerType::RequestType>();
    message->ParseFromString(serialized_message);
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        CHECK(client_writer_->Write(*message)) << "Write failed.";
        break;
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      case ::grpc::internal::RpcMethod::NORMAL_RPC:
        LOG(FATAL) << "Not implemented";
        break;
    }
    WaitForHandlerCompletion(RpcHandlerWrapper::ON_REQUEST);
  }

  // Sends a WRITES_DONE event to the handler, waits for the handler
  // to finish processing the READS_DONE event before returning.
  void SendWritesDone() {
    auto message = cartographer::common::make_unique<
        typename RpcHandlerType::RequestType>();
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        CHECK(client_writer_->WritesDone()) << "WritesDone failed.";
        break;
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      case ::grpc::internal::RpcMethod::NORMAL_RPC:
        LOG(FATAL) << "Not implemented";
        break;
    }
    WaitForHandlerCompletion(RpcHandlerWrapper::ON_READS_DONE);
  }

  // Sends a FINISH event to the handler under test, waits for the
  // handler to finish processing the event before returning.
  void SendFinish() {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        CHECK(client_writer_->Finish()) << "Finish failed.";
        break;
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      case ::grpc::internal::RpcMethod::NORMAL_RPC:
        LOG(FATAL) << "Not implemented";
        break;
    }
    WaitForHandlerCompletion(RpcHandlerWrapper::ON_FINISH);
  }

 private:
  using ClientWriter = ::grpc::internal::ClientWriterFactory<
      typename RpcHandlerType::RequestType>;

  void WaitForHandlerCompletion(RpcHandlerWrapper::RpcHandlerEvent event) {
    CHECK_EQ(rpc_handler_event_queue_.Pop(), event);
  }

  void InstantiateReadersAndWriters() {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        if (!response_) {
          response_ = cartographer::common::make_unique<
              typename RpcHandlerType::ResponseType>();
        }
        if (!client_writer_) {
          client_writer_ = CreateClientWriter();
        }
        break;
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
      case ::grpc::internal::RpcMethod::NORMAL_RPC:
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
        LOG(FATAL) << "Not implemented";
        break;
    }
  }

  std::unique_ptr<ClientWriter> CreateClientWriter() {
    return std::unique_ptr<ClientWriter>(ClientWriter::Create(
        channel_.get(), rpc_method_, &context_, response_.get()));
  }

  RpcHandlerInfo GetRpcHandlerInfo() {
    ::grpc::internal::RpcMethod::RpcType rpc_type =
        RpcType<typename RpcHandlerType::IncomingType,
                typename RpcHandlerType::OutgoingType>::value;
    auto event_callback =
        [this](RpcHandlerWrapper<RpcHandlerType>::RpcHandlerEvent event) {
          rpc_handler_event_queue_.Push(event);
        };
    auto handler_instantiator = [this](
                                    Rpc *const rpc,
                                    ExecutionContext *const execution_context) {
      std::unique_ptr<RpcHandlerInterface> rpc_handler =
          cartographer::common::make_unique<RpcHandlerWrapper<RpcHandlerType>>(
              event_callback);
      rpc_handler->SetRpc(rpc);
      rpc_handler->SetExecutionContext(execution_context);
      return rpc_handler;
    };
    return RpcHandlerInfo{
        RpcHandlerType::RequestType::default_instance().GetDescriptor(),
        RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
        handler_instantiator, rpc_type, kFullyQualifiedMethodName};
  }

  ::grpc::internal::RpcMethod rpc_method_;
  ::grpc::ClientContext context_;
  std::shared_ptr<::grpc::ChannelInterface> channel_;
  std::unique_ptr<::grpc::ClientWriter<typename RpcHandlerType::RequestType>>
      client_writer_;
  std::unique_ptr<typename RpcHandlerType::ResponseType> response_;
  cartographer::common::BlockingQueue<RpcHandlerWrapper::RpcHandlerEvent>
      rpc_handler_event_queue_;
};

}  // namespace testing
}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_
