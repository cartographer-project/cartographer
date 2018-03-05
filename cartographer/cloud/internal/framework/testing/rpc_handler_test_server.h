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

#ifndef CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_
#define CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_

#include <functional>
#include <string>

#include "cartographer/cloud/internal/framework/client.h"
#include "cartographer/cloud/internal/framework/rpc_handler_interface.h"
#include "cartographer/cloud/internal/framework/server.h"
#include "cartographer/cloud/internal/framework/testing/rpc_handler_wrapper.h"
#include "cartographer/common/blocking_queue.h"
#include "grpc++/grpc++.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace framework {
namespace testing {

namespace {
const std::string kServerAddress = "localhost:50051";
}  // namespace

template <typename RpcHandlerType>
class RpcHandlerTestServer : public Server {
 public:
  RpcHandlerTestServer(std::unique_ptr<ExecutionContext> execution_context)
      : Server(Options{1, 1, kServerAddress}),
        channel_(::grpc::CreateChannel(kServerAddress,
                                       ::grpc::InsecureChannelCredentials())),
        client_(channel_) {
    std::string method_full_name_under_test =
        RpcHandlerInterface::Instantiate<RpcHandlerType>()->method_name();
    std::string service_full_name;
    std::string method_name;
    std::tie(service_full_name, method_name) =
        Server::Builder::ParseMethodFullName(method_full_name_under_test);
    this->AddService(
        service_full_name,
        {{method_name, GetRpcHandlerInfo(method_full_name_under_test)}});
    this->SetExecutionContext(std::move(execution_context));
    this->Start();
  }

  ~RpcHandlerTestServer() { this->Shutdown(); };

  void SendWrite(const typename RpcHandlerType::RequestType &message) {
    EXPECT_TRUE(client_.Write(message));
    WaitForHandlerCompletion(RpcHandlerWrapper<RpcHandlerType>::ON_REQUEST);
  }

  // Parses a request message from the passed string and issues the
  // request against the handler, waits for the handler to complete
  // processing before returning.
  void SendWrite(const std::string &serialized_message) {
    typename RpcHandlerType::RequestType message;
    message.ParseFromString(serialized_message);
    Write(message);
  }

  // Sends a WRITES_DONE event to the handler, waits for the handler
  // to finish processing the READS_DONE event before returning.
  void SendWritesDone() {
    EXPECT_TRUE(client_.WritesDone());
    WaitForHandlerCompletion(RpcHandlerWrapper<RpcHandlerType>::ON_READS_DONE);
  }

  // Sends a FINISH event to the handler under test, waits for the
  // handler to finish processing the event before returning.
  void SendFinish() {
    EXPECT_TRUE(client_.Finish().ok());
    WaitForHandlerCompletion(RpcHandlerWrapper<RpcHandlerType>::ON_FINISH);
  }

  const typename RpcHandlerType::ResponseType &response() {
    return client_.response();
  }

 private:
  using ClientWriter = ::grpc::internal::ClientWriterFactory<
      typename RpcHandlerType::RequestType>;

  void WaitForHandlerCompletion(
      typename RpcHandlerWrapper<RpcHandlerType>::RpcHandlerEvent event) {
    CHECK_EQ(rpc_handler_event_queue_.Pop(), event);
  }

  RpcHandlerInfo GetRpcHandlerInfo(const std::string &method_full_name) {
    ::grpc::internal::RpcMethod::RpcType rpc_type =
        RpcType<typename RpcHandlerType::IncomingType,
                typename RpcHandlerType::OutgoingType>::value;
    auto event_callback =
        [this](
            typename RpcHandlerWrapper<RpcHandlerType>::RpcHandlerEvent event) {
          rpc_handler_event_queue_.Push(event);
        };
    auto handler_instantiator = [event_callback](
                                    Rpc *const rpc,
                                    ExecutionContext *const execution_context) {
      std::unique_ptr<RpcHandlerInterface> rpc_handler =
          common::make_unique<RpcHandlerWrapper<RpcHandlerType>>(
              event_callback);
      rpc_handler->SetRpc(rpc);
      rpc_handler->SetExecutionContext(execution_context);
      return rpc_handler;
    };
    return RpcHandlerInfo{
        RpcHandlerType::RequestType::default_instance().GetDescriptor(),
        RpcHandlerType::ResponseType::default_instance().GetDescriptor(),
        handler_instantiator, rpc_type, method_full_name};
  }

  std::shared_ptr<::grpc::Channel> channel_;
  cloud::framework::Client<RpcHandlerType> client_;
  common::BlockingQueue<
      typename RpcHandlerWrapper<RpcHandlerType>::RpcHandlerEvent>
      rpc_handler_event_queue_;
};

}  // namespace testing
}  // namespace framework
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_TESTING_RPC_HANDLER_TEST_SERVER_H_
