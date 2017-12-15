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

#include "cartographer_grpc/framework/execution_context.h"
#include "cartographer_grpc/framework/proto/math_service.grpc.pb.h"
#include "cartographer_grpc/framework/proto/math_service.pb.h"
#include "cartographer_grpc/framework/rpc_handler.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace framework {
namespace {

class MathServerContext : public ExecutionContext {
 public:
  int additional_increment() { return 10; }
};

class GetSumHandler
    : public RpcHandler<Stream<proto::GetSumRequest>, proto::GetSumResponse> {
 public:
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
  void OnRequest(const proto::GetSquareRequest& request) override {
    auto response =
        cartographer::common::make_unique<proto::GetSquareResponse>();
    response->set_output(request.input() * request.input());
    Send(std::move(response));
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
    server_builder.RegisterHandler<GetSumHandler, proto::Math>("GetSum");
    server_builder.RegisterHandler<GetSquareHandler, proto::Math>("GetSquare");
    server_builder.RegisterHandler<GetRunningSumHandler, proto::Math>(
        "GetRunningSum");
    server_ = server_builder.Build();

    client_channel_ =
        grpc::CreateChannel(kServerAddress, grpc::InsecureChannelCredentials());
    stub_ = proto::Math::NewStub(client_channel_);
  }

  std::unique_ptr<Server> server_;
  std::shared_ptr<grpc::Channel> client_channel_;
  std::unique_ptr<proto::Math::Stub> stub_;
  grpc::ClientContext client_context_;
};

TEST_F(ServerTest, StartAndStopServerTest) {
  server_->Start();
  server_->Shutdown();
}

TEST_F(ServerTest, ProcessRpcStreamTest) {
  server_->SetExecutionContext(
      cartographer::common::make_unique<MathServerContext>());
  server_->Start();

  proto::GetSumResponse result;
  std::unique_ptr<grpc::ClientWriter<proto::GetSumRequest>> writer(
      stub_->GetSum(&client_context_, &result));
  for (int i = 0; i < 3; ++i) {
    proto::GetSumRequest request;
    request.set_input(i);
    EXPECT_TRUE(writer->Write(request));
  }
  writer->WritesDone();
  grpc::Status status = writer->Finish();
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(result.output(), 33);

  server_->Shutdown();
}

TEST_F(ServerTest, ProcessUnaryRpcTest) {
  server_->Start();

  proto::GetSquareResponse result;
  proto::GetSquareRequest request;
  request.set_input(11);
  grpc::Status status = stub_->GetSquare(&client_context_, request, &result);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(result.output(), 121);

  server_->Shutdown();
}

TEST_F(ServerTest, ProcessBidiStreamingRpcTest) {
  server_->Start();

  auto reader_writer = stub_->GetRunningSum(&client_context_);
  for (int i = 0; i < 3; ++i) {
    proto::GetSumRequest request;
    request.set_input(i);
    EXPECT_TRUE(reader_writer->Write(request));
  }
  reader_writer->WritesDone();
  proto::GetSumResponse response;

  std::list<int> expected_responses = {0, 0, 1, 1, 3, 3};
  while (reader_writer->Read(&response)) {
    EXPECT_EQ(expected_responses.front(), response.output());
    expected_responses.pop_front();
  }
  EXPECT_TRUE(expected_responses.empty());

  server_->Shutdown();
}

}  // namespace
}  // namespace framework
}  // namespace cartographer_grpc
