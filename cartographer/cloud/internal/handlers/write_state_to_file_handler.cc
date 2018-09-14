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

#include "cartographer/cloud/internal/handlers/write_state_to_file_handler.h"

#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/io/proto_stream.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void WriteStateToFileHandler::OnRequest(
    const proto::WriteStateToFileRequest& request) {
  if (request.filename().empty()) {
    Finish(::grpc::Status(::grpc::INVALID_ARGUMENT, "Filename empty."));
    return;
  }
  GetContext<MapBuilderContextInterface>()->map_builder().SerializeStateToFile(
      /*include_unfinished_submaps=*/false, request.filename());
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
