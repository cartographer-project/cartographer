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

#ifndef CARTOGRAPHER_GRPC_FRAMEWORK_TYPES_H
#define CARTOGRAPHER_GRPC_FRAMEWORK_TYPES_H

#include <grpc++/grpc++.h>

namespace cartographer_grpc {
namespace framework {

template <typename Request>
class Stream {
  using type = Request;
};

template <template <typename> class, typename T>
struct Strip {
  using type = T;
};

template <template <typename> class T, typename Param>
struct Strip<T, T<Param>> {
  using type = Param;
};

template <typename T>
using StripStream = typename Strip<Stream, T>::type;

template <typename Incoming, typename Outgoing>
struct RpcType
    : public std::integral_constant<grpc::internal::RpcMethod::RpcType,
                                    grpc::internal::RpcMethod::NORMAL_RPC> {};

template <typename Incoming, typename Outgoing>
struct RpcType<Stream<Incoming>, Outgoing>
    : public std::integral_constant<
          grpc::internal::RpcMethod::RpcType,
          grpc::internal::RpcMethod::CLIENT_STREAMING> {};

template <typename Incoming, typename Outgoing>
struct RpcType<Incoming, Stream<Outgoing>>
    : public std::integral_constant<
          grpc::internal::RpcMethod::RpcType,
          grpc::internal::RpcMethod::SERVER_STREAMING> {};

template <typename Incoming, typename Outgoing>
struct RpcType<Stream<Incoming>, Stream<Outgoing>>
    : public std::integral_constant<grpc::internal::RpcMethod::RpcType,
                                    grpc::internal::RpcMethod::BIDI_STREAMING> {
};

}  // namespace framework
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_FRAMEWORK_TYPES_H
