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

#include "cartographer/sensor/ordered_multi_queue.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

class OrderedMultiQueueTest : public ::testing::Test {
 protected:
  const QueueKey kFirst{1, "foo"};
  const QueueKey kSecond{1, "bar"};
  const QueueKey kThird{2, "bar"};

  void SetUp() {
    for (const auto& queue_key : {kFirst, kSecond, kThird}) {
      queue_.AddQueue(queue_key, [this](const common::Time time,
                                        std::unique_ptr<Data> data) {
        EXPECT_EQ(common::ToUniversal(time), data->imu.linear_acceleration.x());
        if (!values_.empty()) {
          EXPECT_GT(data->imu.linear_acceleration.x(),
                    values_.back().imu.linear_acceleration.x());
        }
        values_.push_back(*data);
      });
    }
  }

  std::unique_ptr<Data> MakeImu(const int ordinal) {
    return common::make_unique<Data>(
        "unused_frame_id",
        Data::Imu{ordinal * Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()});
  }

  std::vector<Data> values_;
  OrderedMultiQueue queue_;
};

TEST_F(OrderedMultiQueueTest, Ordering) {
  queue_.Add(kFirst, common::FromUniversal(4), MakeImu(4));
  queue_.Add(kFirst, common::FromUniversal(5), MakeImu(5));
  queue_.Add(kFirst, common::FromUniversal(6), MakeImu(6));
  EXPECT_TRUE(values_.empty());
  queue_.Add(kSecond, common::FromUniversal(1), MakeImu(1));
  EXPECT_TRUE(values_.empty());
  queue_.Add(kThird, common::FromUniversal(2), MakeImu(2));
  EXPECT_EQ(values_.size(), 1);
  queue_.Add(kSecond, common::FromUniversal(3), MakeImu(3));
  EXPECT_EQ(values_.size(), 2);
  queue_.Add(kSecond, common::FromUniversal(7), MakeImu(7));
  queue_.Add(kThird, common::FromUniversal(8), MakeImu(8));
  queue_.Flush();

  EXPECT_EQ(8, values_.size());
  for (size_t i = 0; i < values_.size(); ++i) {
    EXPECT_EQ(i + 1, values_[i].imu.linear_acceleration.x());
  }
}

TEST_F(OrderedMultiQueueTest, MarkQueueAsFinished) {
  queue_.Add(kFirst, common::FromUniversal(1), MakeImu(1));
  queue_.Add(kFirst, common::FromUniversal(2), MakeImu(2));
  queue_.Add(kFirst, common::FromUniversal(3), MakeImu(3));
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kFirst);
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kThird);

  EXPECT_EQ(3, values_.size());
  for (size_t i = 0; i < values_.size(); ++i) {
    EXPECT_EQ(i + 1, values_[i].imu.linear_acceleration.x());
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
