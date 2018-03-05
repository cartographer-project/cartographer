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

#include "cartographer/cloud/internal/testing/test_helpers.h"

namespace cartographer {
namespace cloud {
namespace testing {

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddImuDataRequest>(
    const proto::AddImuDataRequest &proto) {
  return [proto](const sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const sensor::Dispatchable<sensor::ImuData> *>(&data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               sensor::ToProto(dispatchable->data()), proto.imu_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddFixedFramePoseDataRequest>(
    const proto::AddFixedFramePoseDataRequest &proto) {
  return [proto](const sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const sensor::Dispatchable<sensor::FixedFramePoseData> *>(
            &data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               sensor::ToProto(dispatchable->data()),
               proto.fixed_frame_pose_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddOdometryDataRequest>(
    const proto::AddOdometryDataRequest &proto) {
  return [proto](const sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const sensor::Dispatchable<sensor::OdometryData> *>(&data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               sensor::ToProto(dispatchable->data()), proto.odometry_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddLandmarkDataRequest>(
    const proto::AddLandmarkDataRequest &proto) {
  return [proto](const sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const sensor::Dispatchable<sensor::LandmarkData> *>(&data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               sensor::ToProto(dispatchable->data()), proto.landmark_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

template <>
DataPredicateType BuildDataPredicateEquals<proto::AddRangefinderDataRequest>(
    const proto::AddRangefinderDataRequest &proto) {
  return [proto](const sensor::Data &data) {
    const auto *dispatchable =
        dynamic_cast<const sensor::Dispatchable<sensor::TimedPointCloudData> *>(
            &data);
    CHECK_NOTNULL(dispatchable);
    return google::protobuf::util::MessageDifferencer::Equals(
               sensor::ToProto(dispatchable->data()),
               proto.timed_point_cloud_data()) &&
           dispatchable->GetSensorId() == proto.sensor_metadata().sensor_id();
  };
}

ProtoPredicateType BuildProtoPredicateEquals(
    const google::protobuf::Message *proto) {
  return [proto](const google::protobuf::Message &message) {
    return google::protobuf::util::MessageDifferencer::Equals(*proto, message);
  };
}

}  // namespace testing
}  // namespace cloud
}  // namespace cartographer
