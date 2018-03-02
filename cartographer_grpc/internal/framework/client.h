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

#ifndef CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_CLIENT_H
#define CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_CLIENT_H

#include "cartographer_grpc/internal/framework/retry.h"
#include "cartographer_grpc/internal/framework/rpc_handler_interface.h"
#include "cartographer_grpc/internal/framework/type_traits.h"
#include "glog/logging.h"
#include "grpc++/grpc++.h"
#include "grpc++/impl/codegen/client_unary_call.h"
#include "grpc++/impl/codegen/sync_stream.h"

namespace cartographer {
namespace cloud {
namespace framework {

template <typename RpcHandlerType>
class Client {
 public:
  Client(std::shared_ptr<::grpc::Channel> channel, RetryStrategy retry_strategy)
      : channel_(channel),
        client_context_(common::make_unique<::grpc::ClientContext>()),
        rpc_method_name_(
            RpcHandlerInterface::Instantiate<RpcHandlerType>()->method_name()),
        rpc_method_(rpc_method_name_.c_str(),
                    RpcType<typename RpcHandlerType::IncomingType,
                            typename RpcHandlerType::OutgoingType>::value,
                    channel_),
        retry_strategy_(retry_strategy) {
    CHECK(!retry_strategy ||
          rpc_method_.method_type() == ::grpc::internal::RpcMethod::NORMAL_RPC)
        << "Retry is currently only support for NORMAL_RPC.";
  }

  Client(std::shared_ptr<::grpc::Channel> channel)
      : channel_(channel),
        client_context_(common::make_unique<::grpc::ClientContext>()),
        rpc_method_name_(
            RpcHandlerInterface::Instantiate<RpcHandlerType>()->method_name()),
        rpc_method_(rpc_method_name_.c_str(),
                    RpcType<typename RpcHandlerType::IncomingType,
                            typename RpcHandlerType::OutgoingType>::value,
                    channel_) {}

  bool Read(typename RpcHandlerType::ResponseType *response) {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
        InstantiateClientReaderWriterIfNeeded();
        return client_reader_writer_->Read(response);
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
        CHECK(client_reader_);
        return client_reader_->Read(response);
      default:
        LOG(FATAL) << "Not implemented.";
    }
  }

  bool Write(const typename RpcHandlerType::RequestType &request) {
    return RetryWithStrategy(
        retry_strategy_,
        std::bind(&Client<RpcHandlerType>::WriteImpl, this, request),
        std::bind(&Client<RpcHandlerType>::Reset, this));
  }

  bool WritesDone() {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        InstantiateClientWriterIfNeeded();
        return client_writer_->WritesDone();
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
        InstantiateClientReaderWriterIfNeeded();
        return client_reader_writer_->WritesDone();
      default:
        LOG(FATAL) << "Not implemented.";
    }
  }

  ::grpc::Status Finish() {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        InstantiateClientWriterIfNeeded();
        return client_writer_->Finish();
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
        InstantiateClientReaderWriterIfNeeded();
        return client_reader_writer_->Finish();
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
        CHECK(client_reader_);
        return client_reader_->Finish();
      default:
        LOG(FATAL) << "Not implemented.";
    }
  }

  const typename RpcHandlerType::ResponseType &response() {
    CHECK(rpc_method_.method_type() ==
              ::grpc::internal::RpcMethod::NORMAL_RPC ||
          rpc_method_.method_type() ==
              ::grpc::internal::RpcMethod::CLIENT_STREAMING);
    return response_;
  }

 private:
  void Reset() {
    client_context_ = common::make_unique<::grpc::ClientContext>();
  }

  bool WriteImpl(const typename RpcHandlerType::RequestType &request) {
    switch (rpc_method_.method_type()) {
      case ::grpc::internal::RpcMethod::NORMAL_RPC:
        return MakeBlockingUnaryCall(request, &response_).ok();
      case ::grpc::internal::RpcMethod::CLIENT_STREAMING:
        InstantiateClientWriterIfNeeded();
        return client_writer_->Write(request);
      case ::grpc::internal::RpcMethod::BIDI_STREAMING:
        InstantiateClientReaderWriterIfNeeded();
        return client_reader_writer_->Write(request);
      case ::grpc::internal::RpcMethod::SERVER_STREAMING:
        InstantiateClientReader(request);
        return true;
    }
    LOG(FATAL) << "Not reached.";
  }

  void InstantiateClientWriterIfNeeded() {
    CHECK_EQ(rpc_method_.method_type(),
             ::grpc::internal::RpcMethod::CLIENT_STREAMING);
    if (!client_writer_) {
      client_writer_.reset(
          ::grpc::internal::
              ClientWriterFactory<typename RpcHandlerType::RequestType>::Create(
                  channel_.get(), rpc_method_, client_context_.get(),
                  &response_));
    }
  }

  void InstantiateClientReaderWriterIfNeeded() {
    CHECK_EQ(rpc_method_.method_type(),
             ::grpc::internal::RpcMethod::BIDI_STREAMING);
    if (!client_reader_writer_) {
      client_reader_writer_.reset(
          ::grpc::internal::ClientReaderWriterFactory<
              typename RpcHandlerType::RequestType,
              typename RpcHandlerType::ResponseType>::Create(channel_.get(),
                                                             rpc_method_,
                                                             client_context_
                                                                 .get()));
    }
  }

  void InstantiateClientReader(
      const typename RpcHandlerType::RequestType &request) {
    CHECK_EQ(rpc_method_.method_type(),
             ::grpc::internal::RpcMethod::SERVER_STREAMING);
    client_reader_.reset(
        ::grpc::internal::
            ClientReaderFactory<typename RpcHandlerType::ResponseType>::Create(
                channel_.get(), rpc_method_, client_context_.get(), request));
  }

  ::grpc::Status MakeBlockingUnaryCall(
      const typename RpcHandlerType::RequestType &request,
      typename RpcHandlerType::ResponseType *response) {
    CHECK_EQ(rpc_method_.method_type(),
             ::grpc::internal::RpcMethod::NORMAL_RPC);
    return ::grpc::internal::BlockingUnaryCall(
        channel_.get(), rpc_method_, client_context_.get(), request, response);
  }

  std::shared_ptr<::grpc::Channel> channel_;
  std::unique_ptr<::grpc::ClientContext> client_context_;
  const std::string rpc_method_name_;
  const ::grpc::internal::RpcMethod rpc_method_;

  std::unique_ptr<::grpc::ClientWriter<typename RpcHandlerType::RequestType>>
      client_writer_;
  std::unique_ptr<
      ::grpc::ClientReaderWriter<typename RpcHandlerType::RequestType,
                                 typename RpcHandlerType::ResponseType>>
      client_reader_writer_;
  std::unique_ptr<::grpc::ClientReader<typename RpcHandlerType::ResponseType>>
      client_reader_;
  typename RpcHandlerType::ResponseType response_;
  RetryStrategy retry_strategy_;
};

}  // namespace framework
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_GRPC_INTERNAL_FRAMEWORK_CLIENT_H
