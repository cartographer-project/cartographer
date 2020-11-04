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

#include "cartographer/io/internal/testing/test_helpers.h"

#include "cartographer/mapping/proto/serialization.pb.h"

namespace cartographer {
namespace io {
namespace testing {

std::unique_ptr<InMemoryProtoStreamReader> ProtoReaderFromStrings(
    const std::string &header_textpb,
    const std::initializer_list<std::string> &data_textpbs) {
  std::queue<std::unique_ptr<::google::protobuf::Message>> proto_queue;
  proto_queue.emplace(absl::make_unique<
                      ::cartographer::mapping::proto::SerializationHeader>(
      ProtoFromStringOrDie<::cartographer::mapping::proto::SerializationHeader>(
          header_textpb)));
  for (const std::string &data_textpb : data_textpbs) {
    proto_queue.emplace(
        absl::make_unique<::cartographer::mapping::proto::SerializedData>(
            ProtoFromStringOrDie<
                ::cartographer::mapping::proto::SerializedData>(data_textpb)));
  }
  return absl::make_unique<InMemoryProtoStreamReader>(std::move(proto_queue));
}

}  // namespace testing
}  // namespace io
}  // namespace cartographer
