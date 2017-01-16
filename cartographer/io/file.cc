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

#include "cartographer/io/file.h"

namespace cartographer {
namespace io {

IoStreamFile::IoStreamFile(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {}

IoStreamFile::~IoStreamFile() {}

bool IoStreamFile::Write(const char* data, size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.write(data, len);
  return !out_.bad();
}

bool IoStreamFile::Close() {
  if (out_.bad()) {
    return false;
  }
  out_.close();
  return !out_.bad();
}

bool IoStreamFile::SeekToStart() {
  if (out_.bad()) {
    return false;
  }
  out_.flush();
  out_.seekp(0);
  return !out_.bad();
}

}  // namespace io
}  // namespace cartographer
