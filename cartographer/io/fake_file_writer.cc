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
#include "gtest/gtest.h"

namespace cartographer {
namespace io {

FakeFileWriter::FakeFileWriter(const std::string& filename) : filename_(filename), was_closed_(false){
}

FakeFileWriter::~FakeFileWriter() {
  EXPECT_TRUE(was_closed_);
}

bool FakeFileWriter::Write(const char* const data, const size_t len) {
  
  EXPECT_FALSE(was_closed_);
  out_.write(data, len);
  return true;
}

bool FakeFileWriter::Close() { 
  was_closed_ = true; 
  return true; 
}

bool FakeFileWriter::WriteHeader(const char* const data, const size_t len) {
  out_.flush();
  out_.seekp(0);
  return Write(data, len);
}

std::string FakeFileWriter::GetFilename() { return filename_; }

std::string FakeFileWriter::GetOutput() const { 
  EXPECT_TRUE(was_closed_);
  return out_; 
}

}  // namespace io
}  // namespace cartographer
