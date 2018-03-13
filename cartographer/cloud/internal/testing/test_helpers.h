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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_TESTING_TEST_HELPERS_H
#define CARTOGRAPHER_CLOUD_INTERNAL_TESTING_TEST_HELPERS_H

#include "cartographer/cloud/proto/map_builder_service.pb.h"
#include "cartographer/sensor/internal/dispatchable.h"
#include "google/protobuf/util/message_differencer.h"

namespace cartographer {
namespace cloud {
namespace testing {

using DataPredicateType = std::function<bool(const sensor::Data &)>;
using ProtoPredicateType =
    std::function<bool(const google::protobuf::Message &)>;

template <typename T>
DataPredicateType BuildDataPredicateEquals(const T &proto);

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddImuDataRequest>(
    const proto::AddImuDataRequest &proto);
template <>
DataPredicateType BuildDataPredicateEquals<proto::AddFixedFramePoseDataRequest>(
    const proto::AddFixedFramePoseDataRequest &proto);
template <>
DataPredicateType BuildDataPredicateEquals<proto::AddOdometryDataRequest>(
    const proto::AddOdometryDataRequest &proto);
template <>
DataPredicateType BuildDataPredicateEquals<proto::AddLandmarkDataRequest>(
    const proto::AddLandmarkDataRequest &proto);
template <>
DataPredicateType BuildDataPredicateEquals<proto::AddRangefinderDataRequest>(
    const proto::AddRangefinderDataRequest &proto);

ProtoPredicateType BuildProtoPredicateEquals(
    const google::protobuf::Message *proto);

}  // namespace testing
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_TESTING_TEST_HELPERS_H
