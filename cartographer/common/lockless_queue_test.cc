#include "cartographer/common/lockless_queue.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(LocklessQueueTest, PushAndPop) {
  LocklessQueue<int> queue;
  queue.Push(absl::make_unique<int>(1));
  queue.Push(absl::make_unique<int>(2));
  EXPECT_EQ(*queue.Pop(), 1);
  queue.Push(absl::make_unique<int>(3));
  queue.Push(absl::make_unique<int>(4));
  EXPECT_EQ(*queue.Pop(), 2);
  queue.Push(absl::make_unique<int>(5));
  EXPECT_EQ(*queue.Pop(), 3);
  EXPECT_EQ(*queue.Pop(), 4);
  EXPECT_EQ(*queue.Pop(), 5);
  EXPECT_EQ(queue.Pop(), nullptr);
}

}  // namespace
}  // namespace common
}  // namespace cartographer