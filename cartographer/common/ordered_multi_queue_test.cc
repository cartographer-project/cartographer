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

#include "cartographer/common/ordered_multi_queue.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(OrderedMultiQueue, Ordering) {
  std::vector<int> values;
  OrderedMultiQueue<int, int, int> queue;
  for (int i : {1, 2, 3}) {
    queue.AddQueue(i, [&values](std::unique_ptr<int> value) {
      if (!values.empty()) {
        EXPECT_GT(*value, values.back());
      }
      values.push_back(*value);
    });
  }
  queue.Add(1, 4, common::make_unique<int>(4));
  queue.Add(1, 5, common::make_unique<int>(5));
  queue.Add(1, 6, common::make_unique<int>(6));
  EXPECT_TRUE(values.empty());
  queue.Add(2, 1, common::make_unique<int>(1));
  EXPECT_TRUE(values.empty());
  queue.Add(3, 2, common::make_unique<int>(2));
  EXPECT_EQ(values.size(), 1);
  queue.Add(2, 3, common::make_unique<int>(3));
  EXPECT_EQ(values.size(), 2);
  queue.Add(2, 7, common::make_unique<int>(7));
  queue.Add(3, 8, common::make_unique<int>(8));
  queue.Flush();

  EXPECT_EQ(8, values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(i + 1, values[i]);
  }
}

TEST(OrderedMultiQueue, MarkQueueAsFinished) {
  std::vector<int> values;
  OrderedMultiQueue<int, int, int> queue;
  for (int i : {1, 2, 3}) {
    queue.AddQueue(i, [&values](std::unique_ptr<int> value) {
      if (!values.empty()) {
        EXPECT_GT(*value, values.back());
      }
      values.push_back(*value);
    });
  }
  queue.Add(1, 1, common::make_unique<int>(1));
  queue.Add(1, 2, common::make_unique<int>(2));
  queue.Add(1, 3, common::make_unique<int>(3));
  EXPECT_TRUE(values.empty());
  queue.MarkQueueAsFinished(1);
  EXPECT_TRUE(values.empty());
  queue.MarkQueueAsFinished(2);
  EXPECT_TRUE(values.empty());
  queue.MarkQueueAsFinished(3);

  EXPECT_EQ(3, values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(i + 1, values[i]);
  }
}

}  // namespace
}  // namespace common
}  // namespace cartographer
