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

#ifndef CARTOGRAPHER_IO_FILE_H_
#define CARTOGRAPHER_IO_FILE_H_

#include <fstream>
#include <functional>
#include <memory>

#include "cartographer/common/port.h"

namespace cartographer {
namespace io {

// Simple abstraction for a file.
class File {
 public:
  File() {}
  File(const File&) = delete;
  File& operator=(const File&) = delete;

  virtual ~File() {}

  virtual bool Write(const char* data, size_t len) = 0;
  virtual bool Close() = 0;
  virtual bool SeekToStart() = 0;
};

// An Implementation of file using std::ofstream.
class IoStreamFile : public File {
 public:
  virtual ~IoStreamFile() override;

  IoStreamFile(const string& filename);

  bool Write(const char* data, size_t len) override;
  bool Close() override;
  bool SeekToStart() override;

 private:
  std::ofstream out_;
};

using FileFactory =
    std::function<std::unique_ptr<File>(const string& filename)>;

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FILE_H_
