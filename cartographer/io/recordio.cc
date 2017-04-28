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

#include "cartographer/io/recordio.h"

namespace cartographer {
namespace io {

namespace {

const size_t kMagic = 0x7b1d1f7b5bf501db;

void WriteSizeAsLittleEndian(size_t size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

bool ReadSizeAsLittleEndian(std::istream* in, size_t* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<size_t>(in->get()) << 56;
  }
  return *in;
}

}  // namespace

RecordWriter::RecordWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {}

RecordWriter::~RecordWriter() {}

void RecordWriter::Write(const string& uncompressed_data) {
  WriteSizeAsLittleEndian(kMagic, &out_);
  string compressed_data;
  common::FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

bool RecordWriter::Close() {
  out_.close();
  return out_;
}

RecordReader::RecordReader(const string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {}

RecordReader::~RecordReader() {}

bool RecordReader::Read(string* decompressed_data) {
  size_t magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    return false;
  }
  size_t compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

}  // namespace io
}  // namespace cartographer
