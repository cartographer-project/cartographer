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

#ifndef CARTOGRAPHER_IO_INTERNAL_IN_MEMORY_PROTO_STREAM_H_
#define CARTOGRAPHER_IO_INTERNAL_IN_MEMORY_PROTO_STREAM_H_

#include <queue>

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "google/protobuf/message.h"

namespace cartographer {
namespace io {

class ForwardingProtoStreamWriter
    : public cartographer::io::ProtoStreamWriterInterface {
 public:
  // A callback that is invoked anytime 'WriteProto()' is called on the
  // 'ForwardingProtoStreamWriter'. When 'Close()' is called on the
  // 'ForwardingProtoStreamWriter' the callback is invoked with a 'nullptr'.
  using WriterCallback =
      std::function<bool(const google::protobuf::Message* proto)>;

  explicit ForwardingProtoStreamWriter(WriterCallback writer_callback)
      : writer_callback_(writer_callback) {}
  ~ForwardingProtoStreamWriter() = default;

  void WriteProto(const google::protobuf::Message& proto) override;
  bool Close() override;

 private:
  WriterCallback writer_callback_;
};

class InMemoryProtoStreamReader
    : public cartographer::io::ProtoStreamReaderInterface {
 public:
  explicit InMemoryProtoStreamReader(
      std::queue<std::unique_ptr<google::protobuf::Message>>&& state_chunks)
      : state_chunks_(std::move(state_chunks)) {}
  InMemoryProtoStreamReader() = default;
  ~InMemoryProtoStreamReader() = default;

  InMemoryProtoStreamReader(const InMemoryProtoStreamReader&) = delete;
  InMemoryProtoStreamReader& operator=(const InMemoryProtoStreamReader&) =
      delete;

  template <typename MessageType>
  void AddProto(const MessageType& proto) {
    state_chunks_.push(absl::make_unique<MessageType>(proto));
  }

  bool ReadProto(google::protobuf::Message* proto) override;
  bool eof() const override { return state_chunks_.empty(); }

 private:
  std::queue<std::unique_ptr<google::protobuf::Message>> state_chunks_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_IN_MEMORY_PROTO_STREAM_H_
