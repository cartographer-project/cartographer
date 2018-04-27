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
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

std::string toString(const std::vector<char>& data) {
  return std::string(data.data(), data.size());
}

TEST(FakeFileWriter, Filename) {
  auto content = std::make_shared<std::vector<char>>();
  FakeFileWriter writer("file", content);
  EXPECT_EQ("file", writer.GetFilename());
}

TEST(FakeFileWriter, CloseStream) {
  auto content = std::make_shared<std::vector<char>>();
  FakeFileWriter writer("file", content);
  EXPECT_TRUE(writer.Close());
  EXPECT_EQ("", toString(*content));
}

TEST(FakeFileWriter, WriteHeader) {
  auto content = std::make_shared<std::vector<char>>();
  const std::string header("dummy header");
  const std::string header_2("dummy header 2");
  FakeFileWriter writer("file", content);

  EXPECT_TRUE(writer.WriteHeader(header.c_str(), header.size()));
  EXPECT_EQ("dummy header", toString(*content));

  EXPECT_TRUE(writer.WriteHeader(header_2.c_str(), header_2.size()));
  EXPECT_EQ("dummy header 2", toString(*content));

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ("dummy header 2", toString(*content));
}

TEST(FakeFileWriter, Write) {
  auto content = std::make_shared<std::vector<char>>();
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  FakeFileWriter writer("file", content);

  for (const auto& data : data_stream) {
    EXPECT_TRUE(writer.Write(data.c_str(), data.size()));
  }

  EXPECT_EQ("data 1data 2", toString(*content));
  EXPECT_TRUE(writer.Close());
  EXPECT_EQ("data 1data 2", toString(*content));
}

TEST(FakeFileWriter, HeaderAndWrite) {
  auto content = std::make_shared<std::vector<char>>();
  const std::string header("dummy header");
  const std::vector<std::string> data_stream = {"data 1", "data 2"};
  FakeFileWriter writer("file", content);

  EXPECT_TRUE(writer.WriteHeader(header.c_str(), header.size()));
  EXPECT_EQ("dummy header", toString(*content));

  for (const auto& data : data_stream) {
    EXPECT_TRUE(writer.Write(data.c_str(), data.size()));
  }

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ("dummy headerdata 1data 2", toString(*content));
}

TEST(FakeFileWriter, WriteTerminatedString) {
  auto content = std::make_shared<std::vector<char>>();
  std::vector<char> data_stream = {'d', 'a', 't', 'a', '\0', ' ', '1'};
  FakeFileWriter writer("file", content);
  EXPECT_TRUE(writer.Write(data_stream.data(), data_stream.size()));
  EXPECT_EQ(data_stream, *content);
}

TEST(FakeFileWriter, WriteTerminatedHeaderString) {
  auto content = std::make_shared<std::vector<char>>();
  std::vector<char> header = {'h', 'e', 'a', 'd', '\0', ' ', 'e', 'r'};
  FakeFileWriter writer("file", content);
  EXPECT_TRUE(writer.WriteHeader(header.data(), header.size()));
  EXPECT_EQ(header, *content);
}

TEST(FakeFileWriter, HeaderAndWriteTerminatedString) {
  auto content = std::make_shared<std::vector<char>>();
  std::vector<char> header = {'d', 'a', 't', 'a', '\0', ' ', '1'};
  std::vector<char> data = {'h', 'e', 'a', 'd', '\0', ' ', 'e', 'r'};

  FakeFileWriter writer("file", content);
  EXPECT_TRUE(writer.WriteHeader(header.data(), header.size()));
  EXPECT_EQ(header, *content);

  EXPECT_TRUE(writer.Write(data.data(), data.size()));

  std::vector<char> expected_output = header;
  expected_output.insert(expected_output.end(), data.begin(), data.end());

  EXPECT_EQ(expected_output, *content);

  EXPECT_TRUE(writer.Close());
  EXPECT_EQ(expected_output, *content);
}

}  // namespace
}  // namespace io
}  // namespace cartographer
