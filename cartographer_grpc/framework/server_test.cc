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

#include "cartographer_grpc/framework/server.h"

#include <future>

#include "cartographer_grpc/framework/client.h"
#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/proto/math_service.pb.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "glog/logging.h"
#include "google/protobuf/descriptor.h"
#include "grpc++/grpc++.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace framework {
namespace {

using EchoResponder = std::function<bool()>;
class MathServerContext : public ExecutionContext {
 public:
  int additional_increment() { return 10; }
  std::promise<EchoResponder> echo_responder;
};

class GetSumHandler
    : public RpcHandler<Stream<proto::GetSumRequest>, proto::GetSumResponse> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.framework.proto.Math/GetSum";
  }
  void OnRequest(const proto::GetSumRequest& request) override {
    sum_ += GetContext<MathServerContext>()->additional_increment();
    sum_ += request.input();
  }

  void OnReadsDone() override {
    auto response = cartographer::common::make_unique<proto::GetSumResponse>();
    response->set_output(sum_);
    Send(std::move(response));
  }

 private:
  int sum_ = 0;
};

class GetRunningSumHandler : public RpcHandler<Stream<proto::GetSumRequest>,
                                               Stream<proto::GetSumResponse>> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.framework.proto.Math/GetRunningSum";
  }
  void OnRequest(const proto::GetSumRequest& request) override {
    sum_ += request.input();

    // Respond twice to demonstrate bidirectional streaming.
    auto response = cartographer::common::make_unique<proto::GetSumResponse>();
    response->set_output(sum_);
    Send(std::move(response));
    response = cartographer::common::make_unique<proto::GetSumResponse>();
    response->set_output(sum_);
    Send(std::move(response));
  }

  void OnReadsDone() override { Finish(::grpc::Status::OK); }

 private:
  int sum_ = 0;
};

class GetSquareHandler
    : public RpcHandler<proto::GetSquareRequest, proto::GetSquareResponse> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.framework.proto.Math/GetSquare";
  }
  void OnRequest(const proto::GetSquareRequest& request) override {
    auto response =
        cartographer::common::make_unique<proto::GetSquareResponse>();
    response->set_output(request.input() * request.input());
    Send(std::move(response));
  }
};

class GetEchoHandler
    : public RpcHandler<proto::GetEchoRequest, proto::GetEchoResponse> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.framework.proto.Math/GetEcho";
  }
  void OnRequest(const proto::GetEchoRequest& request) override {
    int value = request.input();
    Writer writer = GetWriter();
    GetContext<MathServerContext>()->echo_responder.set_value(
        [writer, value]() {
          auto response =
              cartographer::common::make_unique<proto::GetEchoResponse>();
          response->set_output(value);
          return writer.Write(std::move(response));
        });
  }
};

class GetSequenceHandler
    : public RpcHandler<proto::GetSequenceRequest,
                        Stream<proto::GetSequenceResponse>> {
 public:
  std::string method_name() const override {
    return "/cartographer_grpc.framework.proto.Math/GetSequence";
  }
  void OnRequest(const proto::GetSequenceRequest& request) override {
    for (int i = 0; i < request.input(); ++i) {
      auto response =
          cartographer::common::make_unique<proto::GetSequenceResponse>();
      response->set_output(i);
      Send(std::move(response));
    }
    Finish(::grpc::Status::OK);
  }
};

// TODO(cschuet): Due to the hard-coded part these tests will become flaky when
// run in parallel. It would be nice to find a way to solve that. gRPC also
// allows to communicate over UNIX domain sockets.
const std::string kServerAddress = "localhost:50051";
const std::size_t kNumThreads = 1;

class ServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Server::Builder server_builder;
    server_builder.SetServerAddress(kServerAddress);
    server_builder.SetNumGrpcThreads(kNumThreads);
    server_builder.SetNumEventThreads(kNumThreads);
    server_builder.RegisterHandler<GetSumHandler>();
    server_builder.RegisterHandler<GetSquareHandler>();
    server_builder.RegisterHandler<GetRunningSumHandler>();
    server_builder.RegisterHandler<GetEchoHandler>();
    server_builder.RegisterHandler<GetSequenceHandler>();
    server_ = server_builder.Build();

    client_channel_ =
        grpc::CreateChannel(kServerAddress, grpc::InsecureChannelCredentials());
  }

  std::unique_ptr<Server> server_;
  std::shared_ptr<grpc::Channel> client_channel_;
};

TEST_F(ServerTest, StartAndStopServerTest) {
  server_->Start();
  server_->Shutdown();
}

TEST_F(ServerTest, ProcessRpcStreamTest) {
  server_->SetExecutionContext(
      cartographer::common::make_unique<MathServerContext>());
  server_->Start();

  Client<GetSumHandler> client(client_channel_);
  for (int i = 0; i < 3; ++i) {
    proto::GetSumRequest request;
    request.set_input(i);
    EXPECT_TRUE(client.Write(request));
  }
  EXPECT_TRUE(client.WritesDone());
  EXPECT_TRUE(client.Finish().ok());
  EXPECT_EQ(client.response().output(), 33);

  server_->Shutdown();
}

TEST_F(ServerTest, ProcessUnaryRpcTest) {
  server_->Start();

  Client<GetSquareHandler> client(client_channel_);
  proto::GetSquareRequest request;
  request.set_input(11);
  EXPECT_TRUE(client.Write(request));
  EXPECT_EQ(client.response().output(), 121);

  server_->Shutdown();
}

TEST_F(ServerTest, ProcessBidiStreamingRpcTest) {
  server_->Start();

  Client<GetRunningSumHandler> client(client_channel_);
  for (int i = 0; i < 3; ++i) {
    proto::GetSumRequest request;
    request.set_input(i);
    EXPECT_TRUE(client.Write(request));
  }
  client.WritesDone();
  proto::GetSumResponse response;
  std::list<int> expected_responses = {0, 0, 1, 1, 3, 3};
  while (client.Read(&response)) {
    EXPECT_EQ(expected_responses.front(), response.output());
    expected_responses.pop_front();
  }
  EXPECT_TRUE(expected_responses.empty());
  EXPECT_TRUE(client.Finish().ok());

  server_->Shutdown();
}

TEST_F(ServerTest, WriteFromOtherThread) {
  server_->SetExecutionContext(
      cartographer::common::make_unique<MathServerContext>());
  server_->Start();

  Server* server = server_.get();
  std::thread response_thread([server]() {
    std::future<EchoResponder> responder_future =
        server->GetContext<MathServerContext>()->echo_responder.get_future();
    responder_future.wait();
    auto responder = responder_future.get();
    CHECK(responder());
  });

  Client<GetEchoHandler> client(client_channel_);
  proto::GetEchoRequest request;
  request.set_input(13);
  EXPECT_TRUE(client.Write(request));
  response_thread.join();
  EXPECT_EQ(client.response().output(), 13);

  server_->Shutdown();
}

TEST_F(ServerTest, ProcessServerStreamingRpcTest) {
  server_->Start();

  Client<GetSequenceHandler> client(client_channel_);
  proto::GetSequenceRequest request;
  request.set_input(12);

  client.Write(request);
  proto::GetSequenceResponse response;
  for (int i = 0; i < 12; ++i) {
    EXPECT_TRUE(client.Read(&response));
    EXPECT_EQ(response.output(), i);
  }
  EXPECT_FALSE(client.Read(&response));
  EXPECT_TRUE(client.Finish().ok());

  server_->Shutdown();
}

}  // namespace
}  // namespace framework
}  // namespace cartographer_grpc
