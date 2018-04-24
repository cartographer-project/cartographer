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

#include "cartographer/io/file_writer.h"
#include "cartographer/common/make_unique.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

StreamWriter::StreamWriter(std::unique_ptr<std::ostream> ostream,
                           const std::string& filename)
    : filename_(filename), is_closed_(false), out_(std::move(ostream)) {}

StreamWriter::~StreamWriter() {}

bool StreamWriter::Write(const char* const data, const size_t len) {
  if (out_->bad() || is_closed_) {
    return false;
  }
  out_->write(data, len);
  return !out_->bad();
}

bool StreamWriter::Close() {
  if (out_->bad() || is_closed_) {
    return false;
  }
  is_closed_ = true;
  return !out_->bad();
}

bool StreamWriter::WriteHeader(const char* const data, const size_t len) {
  if (out_->bad() || is_closed_) {
    return false;
  }
  out_->flush();
  out_->seekp(0);
  return Write(data, len);
}

std::string StreamWriter::GetFilename() { return filename_; }

StreamFileWriter::StreamFileWriter(const std::string filename)
    : StreamWriter(common::make_unique<std::ofstream>(
                       filename, std::ios::out | std::ios::binary),
                   filename) {}

bool StreamFileWriter::Close() {
  static_cast<std::ofstream*>(out_.get())->close();
  return StreamWriter::Close();
}

StreamFileWriter::~StreamFileWriter() {}

bool StreamFileWriter::Write(const char* data, size_t len) {
  if (!static_cast<std::ofstream*>(out_.get())->is_open()) {
    return false;
  }
  return StreamWriter::Write(data, len);
}

}  // namespace io
}  // namespace cartographer
