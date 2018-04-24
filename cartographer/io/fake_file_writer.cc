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

#include "cartographer/io/fake_file_writer.h"

namespace cartographer {
namespace io {

FakeFileWriter::FakeFileWriter(const std::string& filename,
                               std::shared_ptr<std::string> on_close_output)
    : is_closed_(false), on_close_out_(on_close_output), filename_(filename) {}

bool FakeFileWriter::Write(const char* const data, const size_t len) {
  if (is_closed_) return false;
  out_.append(data, len);
  return true;
}

bool FakeFileWriter::Close() {
  if (is_closed_) return false;
  if (on_close_out_) *on_close_out_ = out_;
  is_closed_ = true;
  return true;
}

bool FakeFileWriter::WriteHeader(const char* const data, const size_t len) {
  if (is_closed_) return false;
  if (out_.size() == 0 || out_.size() < len) {
    out_ = "";
    return Write(data, len);
  }
  out_.replace(0, len, data);
  return true;
}

std::string FakeFileWriter::GetFilename() { return filename_; }

std::string FakeFileWriter::GetOutput() const { return out_; }

}  // namespace io
}  // namespace cartographer
