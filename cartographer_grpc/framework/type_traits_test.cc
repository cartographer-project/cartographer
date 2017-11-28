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

#include "cartographer_grpc/framework/type_traits.h"

#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace framework {
namespace {

TEST(TypeTraitsTest, StreamStripping) {
  ::testing::StaticAssertTypeEq<StripStream<Stream<int>>, int>();
  ::testing::StaticAssertTypeEq<StripStream<int>, int>();
}

TEST(TypeTraitsTest, RpcTypes) {
  EXPECT_EQ((RpcType<int, int>::value),
            ::grpc::internal::RpcMethod::NORMAL_RPC);
  EXPECT_EQ((RpcType<Stream<int>, int>::value),
            ::grpc::internal::RpcMethod::CLIENT_STREAMING);
  EXPECT_EQ((RpcType<int, Stream<int>>::value),
            ::grpc::internal::RpcMethod::SERVER_STREAMING);
  EXPECT_EQ((RpcType<Stream<int>, Stream<int>>::value),
            ::grpc::internal::RpcMethod::BIDI_STREAMING);
}

}  // namespace
}  // namespace framework
}  // namespace cartographer_grpc
