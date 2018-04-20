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

#ifndef CARTOGRAPHER_IO_FAKE_FILE_WRITER_H_
#define CARTOGRAPHER_IO_FAKE_FILE_WRITER_H_

#include <string>

#include "cartographer/io/file_writer.h"

namespace cartographer {
namespace io {

// Fakes a FileWriter by just writing the data to a std::string.
class FakeFileWriter : public FileWriter {
 public:
  FakeFileWriter();
  FakeFileWriter(const std::string& file_name);
  ~FakeFileWriter() override;

  bool WriteHeader(const char* data, size_t len) override;
  bool Write(const char* data, size_t len) override;
  bool Close() override;
  std::string GetFilename() override;
  std::string GetOutput() const;

 private:
  std::string out_;
  std::string fake_filename_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FAKE_FILE_WRITER_H_
