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

#include "cartographer/sensor/landmark_data.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

proto::LandmarkData ToProto(const LandmarkData& landmark_data) {
  proto::LandmarkData proto;
  proto.set_timestamp(common::ToUniversal(landmark_data.time));
  for (const Landmark& landmark : landmark_data.landmarks) {
    auto* item = proto.add_landmarks();
    item->set_id(landmark.id);
    *item->mutable_transform() = transform::ToProto(landmark.transform);
    item->set_translation_weight(landmark.translation_weight);
    item->set_rotation_weight(landmark.rotation_weight);
  }
  return proto;
}

LandmarkData FromProto(const proto::LandmarkData& proto) {
  LandmarkData landmark_data;
  landmark_data.time = common::FromUniversal(proto.timestamp());
  for (const auto& item : proto.landmarks()) {
    landmark_data.landmarks.push_back({
        item.id(),
        transform::ToRigid3(item.transform()),
        item.translation_weight(),
        item.rotation_weight(),
    });
  }
  return landmark_data;
}

}  // namespace sensor
}  // namespace cartographer
