/*
 * Copyright 2017 The Cartographer Authors
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
#include <vector>

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

TEST(FakeFileWriter, Write) {
  const std::string header = "dummy header";
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  std::stringstream expected_result;
  
  FakeFileWriter writer;
  writer.WriteHeader(header.c_str(), header.size());
  expected_result << header;
  ASSERT_EQ(expected_result.str(), writer.GetOutput());

  for(const auto& data : data_stream) {
    writer.WriteHeader(data.c_str(), data.size());
    expected_result << data;
  }
  ASSERT_EQ(expected_result.str(), writer.GetOutput());
}

}  // namespace
}  // namespace io
}  // namespace cartographer
