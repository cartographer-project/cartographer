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

#ifndef CARTOGRAPHER_IO_RECORDIO_H_
#define CARTOGRAPHER_IO_RECORDIO_H_

#include <fstream>

#include "cartographer/common/port.h"

namespace cartographer {
namespace io {

// A simple writer of a compressed sequence of protocol buffer messages to a
// file. The format is not intended to be compatible with any other format used
// outside of Cartographer.
class RecordWriter {
 public:
  RecordWriter(const string& filename);
  ~RecordWriter();

  RecordWriter(const RecordWriter&) = delete;
  RecordWriter& operator=(const RecordWriter&) = delete;

  template <typename MessageType>
  void WriteProto(const MessageType& proto) {
    string uncompressed_data;
    proto.SerializeToString(&uncompressed_data);
    Write(uncompressed_data);
  }

  bool Close();

 private:
  void Write(const string& uncompressed_data);

  std::ofstream out_;
};

// A reader of the format produced by RecordWriter.
class RecordReader {
 public:
  RecordReader(const string& filename);
  ~RecordReader();

  RecordReader(const RecordReader&) = delete;
  RecordReader& operator=(const RecordReader&) = delete;

  template <typename MessageType>
  bool ReadProto(MessageType* proto) {
    string decompressed_data;
    return Read(&decompressed_data) &&
           proto->ParseFromString(decompressed_data);
  }

 private:
  bool Read(string* decompressed_data);

  std::ifstream in_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_RECORDIO_H_
