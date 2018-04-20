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

#include "cartographer/io/fake_file_writer.h"

namespace cartographer {
namespace io {

FakeFileWriter::FakeFileWriter() {}

FakeFileWriter::~FakeFileWriter() {}

bool FakeFileWriter::Write(const char* const data, const size_t len) {
  out_.append(data, len);
  return true;
}

bool FakeFileWriter::Close() { return true; }

bool FakeFileWriter::WriteHeader(const char* const data, const size_t len) {
  return Write(data, len);
}

std::string FakeFileWriter::GetFilename() { return ""; }

std::string FakeFileWriter::GetOutput() const { return out_; }

}  // namespace io
}  // namespace cartographer
