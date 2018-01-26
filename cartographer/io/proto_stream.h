/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_PROTO_STREAM_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_H_

#include <fstream>

#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "google/protobuf/message.h"

namespace cartographer {
namespace io {

// A simple writer of a compressed sequence of protocol buffer messages to a
// file. The format is not intended to be compatible with any other format used
// outside of Cartographer.
//
// TODO(whess): Compress the file instead of individual messages for better
// compression performance? Should we use LZ4?
class ProtoStreamWriter : public ProtoStreamWriterInterface {
 public:
  ProtoStreamWriter(const std::string& filename);
  ~ProtoStreamWriter() = default;

  ProtoStreamWriter(const ProtoStreamWriter&) = delete;
  ProtoStreamWriter& operator=(const ProtoStreamWriter&) = delete;

  void WriteProto(const google::protobuf::Message& proto) override;
  bool Close() override;

 private:
  void Write(const std::string& uncompressed_data);

  std::ofstream out_;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReader : public ProtoStreamReaderInterface {
 public:
  explicit ProtoStreamReader(const std::string& filename);
  ~ProtoStreamReader() = default;

  ProtoStreamReader(const ProtoStreamReader&) = delete;
  ProtoStreamReader& operator=(const ProtoStreamReader&) = delete;

  bool ReadProto(google::protobuf::Message* proto) override;
  bool eof() const override;

 private:
  bool Read(std::string* decompressed_data);

  std::ifstream in_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_H_
