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

#include "cartographer/io/internal/in_memory_proto_stream.h"

#include "glog/logging.h"

namespace cartographer {
namespace io {

void ForwardingProtoStreamWriter::WriteProto(
    const google::protobuf::Message& proto) {
  CHECK(writer_callback_(&proto));
}

bool ForwardingProtoStreamWriter::Close() { return writer_callback_(nullptr); }

bool InMemoryProtoStreamReader::ReadProto(google::protobuf::Message* proto) {
  if (eof()) return false;
  proto->CopyFrom(*state_chunks_.front());
  state_chunks_.pop();
  return true;
}

}  // namespace io
}  // namespace cartographer
