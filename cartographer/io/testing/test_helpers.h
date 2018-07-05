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

#ifndef CARTOGRAPHER_IO_TESTING_TEST_HELPERS_H_
#define CARTOGRAPHER_IO_TESTING_TEST_HELPERS_H_

#include <memory>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"

namespace cartographer {
namespace io {
namespace testing {

template <typename T>
T ProtoFromStringOrDie(const std::string &proto_string) {
  T msg;
  CHECK(google::protobuf::TextFormat::ParseFromString(proto_string, &msg));
  return msg;
}

std::unique_ptr<InMemoryProtoStreamReader> ProtoReaderFromStrings(
    const std::string &header_textpb,
    const std::initializer_list<std::string> &data_textpbs);

}  // namespace testing
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_TESTING_TEST_HELPERS_H_
