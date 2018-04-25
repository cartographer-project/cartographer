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

TEST(FakeFileWriter, CloseStream) {
  auto on_close_output = std::make_shared<std::string>();
  FakeFileWriter writer("file", on_close_output);
  EXPECT_EQ(writer.GetFilename(), "file");
  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(*on_close_output, "");
  EXPECT_FALSE(writer.Close());
}

TEST(FakeFileWriter, WriteHeader) {
  auto on_close_output = std::make_shared<std::string>();
  const std::string header("dummy header");
  const std::string header_2("dummy header 2");
  FakeFileWriter writer("file", on_close_output);
  EXPECT_EQ(writer.GetFilename(), "file");

  EXPECT_TRUE(writer.WriteHeader(header.c_str(), header.size()));
  EXPECT_EQ(*on_close_output, "dummy header");

  EXPECT_TRUE(writer.WriteHeader(header_2.c_str(), header_2.size()));
  EXPECT_EQ(*on_close_output, "dummy header 2");

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(*on_close_output, "dummy header 2");
  EXPECT_FALSE(writer.WriteHeader(header.c_str(), header.size()));

  EXPECT_EQ(*on_close_output, "dummy header 2");
}

TEST(FakeFileWriter, Write) {
  auto on_close_output = std::make_shared<std::string>();
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  FakeFileWriter writer("file", on_close_output);
  EXPECT_EQ(writer.GetFilename(), "file");

  for (const auto& data : data_stream) {
    EXPECT_TRUE(writer.Write(data.c_str(), data.size()));
  }

  EXPECT_EQ(*on_close_output, "data 1data 2");
  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(*on_close_output, "data 1data 2");
}

TEST(FakeFileWriter, HeaderAndWrite) {
  auto on_close_output = std::make_shared<std::string>();
  const std::string header("dummy header");
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  FakeFileWriter writer("file", on_close_output);
  EXPECT_EQ(writer.GetFilename(), "file");

  EXPECT_TRUE(writer.WriteHeader(header.c_str(), header.size()));
  EXPECT_EQ(*on_close_output, "dummy header");

  for (const auto& data : data_stream) {
    EXPECT_TRUE(writer.Write(data.c_str(), data.size()));
  }

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(*on_close_output, "dummy headerdata 1data 2");
}

}  // namespace
}  // namespace io
}  // namespace cartographer
