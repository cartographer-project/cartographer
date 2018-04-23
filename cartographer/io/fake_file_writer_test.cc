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

#include <vector>

#include "cartographer/io/fake_file_writer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

TEST(FakeFileWriter, WriteHeader) {
  const std::string header = "dummy header";
  FakeFileWriter writer;
  EXPECT_TRUE(writer.WriteHeader(header.c_str(), header.size()));
  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(writer.GetOutput(), "dummy header");
}

TEST(FakeFileWriter, Write) {
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  FakeFileWriter writer;
  for (const auto& data : data_stream) {
    EXPECT_TRUE(writer.Write(data.c_str(), data.size()));
  }

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(writer.GetOutput(), "data1 data2");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
