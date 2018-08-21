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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_HANDLER_BASE_H
#define CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_HANDLER_BASE_H

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"

namespace cartographer {
namespace cloud {
namespace handlers {

template <typename HandlerSignatureType>
class AddSensorDataHandlerBase
    : public async_grpc::RpcHandler<HandlerSignatureType> {
 public:
  using SensorDataType =
      async_grpc::StripStream<typename HandlerSignatureType::IncomingType>;

  void OnRequest(const SensorDataType& request) override {
    if (!this->template GetContext<
                 cartographer::cloud::MapBuilderContextInterface>()
             ->CheckClientIdForTrajectory(
                 request.sensor_metadata().client_id(),
                 request.sensor_metadata().trajectory_id())) {
      LOG(ERROR) << "Unknown trajectory with ID "
                 << request.sensor_metadata().trajectory_id()
                 << " and client_id " << request.sensor_metadata().client_id();
      this->template Finish(
          ::grpc::Status(::grpc::NOT_FOUND, "Unknown trajectory"));
      return;
    }
    OnSensorData(request);
  }

  virtual void OnSensorData(const SensorDataType& request) = 0;

  void OnReadsDone() override {
    this->template Send(absl::make_unique<google::protobuf::Empty>());
  }
};

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_HANDLERS_ADD_SENSOR_DATA_HANDLER_BASE_H
