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

namespace cartographer {
namespace io {

// A simple writer of a compressed sequence of protocol buffer messages to a
// file. The format is not intended to be compatible with any other format used
// outside of Cartographer.
//
// TODO(whess): Compress the file instead of individual messages for better
// compression performance? Should we use LZ4?
class ProtoStreamWriter {
 public:
  ProtoStreamWriter(const string& filename);
  ~ProtoStreamWriter();

  ProtoStreamWriter(const ProtoStreamWriter&) = delete;
  ProtoStreamWriter& operator=(const ProtoStreamWriter&) = delete;

  // Serializes, compressed and writes the 'proto' to the file.
  template <typename MessageType>
  void WriteProto(const MessageType& proto) {
    string uncompressed_data;
    proto.SerializeToString(&uncompressed_data);
    Write(uncompressed_data);
  }

  // This should be called to check whether writing was successful.
  bool Close();

 private:
  void Write(const string& uncompressed_data);

  std::ofstream out_;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReader {
 public:
  ProtoStreamReader(const string& filename);
  ~ProtoStreamReader();

  ProtoStreamReader(const ProtoStreamReader&) = delete;
  ProtoStreamReader& operator=(const ProtoStreamReader&) = delete;

  template <typename MessageType>
  bool ReadProto(MessageType* proto) {
    string decompressed_data;
    return Read(&decompressed_data) &&
           proto->ParseFromString(decompressed_data);
  }

  bool eof() const;

 private:
  bool Read(string* decompressed_data);

  std::ifstream in_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_H_
