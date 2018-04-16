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

#include "cartographer/mapping/pose_graph.h"

#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/transform/transform.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(PoseGraph, SerializeConstraint) {
  proto::PoseGraph::Constraint expected_constraint = test::CreateFakeConstraint(
      test::CreateFakeNode(1, 2), test::CreateFakeSubmap3D(2, 3));
  ::google::protobuf::RepeatedPtrField<proto::PoseGraph::Constraint>
      constraint_protos;
  *constraint_protos.Add() = expected_constraint;
  PoseGraph::Constraint constraint = FromProto(constraint_protos).front();
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
      expected_constraint, ToProto(constraint)));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
