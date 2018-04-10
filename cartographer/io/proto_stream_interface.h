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

#ifndef CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_

#include "cartographer/common/port.h"
#include "google/protobuf/message.h"

namespace cartographer {
namespace io {

// A writer for writing proto messages to a pbstream.
class ProtoStreamWriterInterface {
 public:
  virtual ~ProtoStreamWriterInterface() {}

  // Serializes, compressed and writes the 'proto' to the file.
  virtual void WriteProto(const google::protobuf::Message& proto) = 0;

  // This should be called to check whether writing was successful.
  virtual bool Close() = 0;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReaderInterface {
 public:
  ProtoStreamReaderInterface() = default;
  virtual ~ProtoStreamReaderInterface() {}

  // Deserialize compressed proto from the pb stream.
  virtual bool ReadProto(google::protobuf::Message* proto) = 0;

  // 'End-of-file' marker for the pb stream.
  virtual bool eof() const = 0;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_
