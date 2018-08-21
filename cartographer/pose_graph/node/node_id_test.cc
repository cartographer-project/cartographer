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

#include "cartographer/pose_graph/node/node_id.h"

#include <sstream>

#include "cartographer/testing/test_helpers.h"

namespace cartographer {
namespace pose_graph {
namespace {

TEST(NodeIdTest, FromProto) {
  proto::NodeId proto;
  proto.set_object_id("some_object");
  proto.set_timestamp(1);

  NodeId node_id(proto);
  EXPECT_EQ(node_id.object_id, "some_object");
  EXPECT_EQ(common::ToUniversal(node_id.time), 1);
}

TEST(NodeIdTest, ToProto) {
  NodeId node_id{"some_object", common::FromUniversal(1)};
  EXPECT_THAT(node_id.ToProto(),
              testing::EqualsProto("object_id: 'some_object' timestamp: 1"));
}

TEST(NodeIdTest, ToString) {
  std::stringstream ss;
  ss << NodeId{"some_object", common::FromUniversal(1)};

  EXPECT_EQ("(object_id: some_object, time: 1)", ss.str());
}

}  // namespace
}  // namespace pose_graph
}  // namespace cartographer
