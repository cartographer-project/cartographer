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

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {

FakeFileWriter::FakeFileWriter(const std::string& filename,
                               std::shared_ptr<std::vector<char>> content)
    : is_closed_(false), content_(content), filename_(filename) {
  CHECK(content != nullptr);
}

bool FakeFileWriter::Write(const char* const data, const size_t len) {
  EXPECT_FALSE(is_closed_);
  content_->insert(content_->end(), data, data + len);
  return true;
}

bool FakeFileWriter::Close() {
  EXPECT_FALSE(is_closed_);
  is_closed_ = true;
  return true;
}

bool FakeFileWriter::WriteHeader(const char* const data, const size_t len) {
  EXPECT_FALSE(is_closed_);
  if (content_->size() < len) {
    content_->resize(len);
  }
  std::copy(data, data + len, content_->begin());
  return true;
}

std::string FakeFileWriter::GetFilename() { return filename_; }

}  // namespace io
}  // namespace cartographer
