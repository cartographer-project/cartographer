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

#ifndef CARTOGRAPHER_TESTING_TEST_HELPERS_H_
#define CARTOGRAPHER_TESTING_TEST_HELPERS_H_

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"

namespace cartographer {
namespace testing {

template <typename ProtoType>
ProtoType ParseProto(const std::string& proto_string) {
  ProtoType proto;
  EXPECT_TRUE(
      ::google::protobuf::TextFormat::ParseFromString(proto_string, &proto));
  return proto;
}

MATCHER_P(EqualsProto, expected_proto_string, "") {
  using ConstProtoType = typename std::remove_reference<decltype(arg)>::type;
  using ProtoType = typename std::remove_cv<ConstProtoType>::type;

  return google::protobuf::util::MessageDifferencer::Equals(
      arg, ParseProto<ProtoType>(expected_proto_string));
}

::testing::Matcher<double> Near(double expected) {
  constexpr double kPrecision = 1e-05;
  return ::testing::DoubleNear(expected, kPrecision);
}

}  // namespace testing
}  // namespace cartographer

#endif  // CARTOGRAPHER_TESTING_TEST_HELPERS_H_
