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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_CLIENT_WRITER_H_
#define CARTOGRAPHER_GRPC_FRAMEWORK_CLIENT_WRITER_H_

#include <memory>

#include "google/protobuf/empty.pb.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace framework {

template <typename RequestType>
struct ClientWriter {
  grpc::ClientContext client_context;
  std::unique_ptr<grpc::ClientWriter<RequestType>> client_writer;
  google::protobuf::Empty response;
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_CLIENT_WRITER_H_
