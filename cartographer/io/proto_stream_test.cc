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

#include "cartographer/io/proto_stream.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

class ProtoStreamTest : public ::testing::Test {
 protected:
  void SetUp() override { test_directory_ = "."; }

  std::string test_directory_;
};

TEST_F(ProtoStreamTest, WriteAndReadBack) {
  const std::string test_file = test_directory_ + "/test_trajectory.pbstream";
  {
    ProtoStreamWriter writer(test_file);
    for (int i = 0; i != 10; ++i) {
      mapping::proto::Trajectory trajectory;
      trajectory.add_node()->set_timestamp(i);
      writer.WriteProto(trajectory);
    }
    ASSERT_TRUE(writer.Close());
  }
  {
    ProtoStreamReader reader(test_file);
    for (int i = 0; i != 10; ++i) {
      mapping::proto::Trajectory trajectory;
      ASSERT_TRUE(reader.ReadProto(&trajectory));
      ASSERT_EQ(1, trajectory.node_size());
      EXPECT_EQ(i, trajectory.node(0).timestamp());
    }
    mapping::proto::Trajectory trajectory;
    EXPECT_FALSE(reader.ReadProto(&trajectory));
  }
  remove(test_file.c_str());
}

}  // namespace
}  // namespace io
}  // namespace cartographer
