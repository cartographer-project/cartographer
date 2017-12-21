#include "cartographer/common/optional.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(OptionalTest, CreateDisengagedObject) {
  const optional<int> o;
  EXPECT_FALSE(o.has_value());
  const optional<float> x;
  EXPECT_FALSE(x.has_value());
}

TEST(OptionalTest, CreateWithValue) {
  const optional<int> a(5);
  EXPECT_TRUE(a.has_value());
  EXPECT_EQ(5, a.value());
}

TEST(OptionalTest, CreateFromOtherOptional) {
  const optional<int> a(5);
  const optional<int> b = a;
  EXPECT_TRUE(a.has_value());
  EXPECT_TRUE(b.has_value());
  EXPECT_EQ(5, a.value());
  EXPECT_EQ(5, b.value());
}

}  // namespace
}  // namespace common
}  // namespace cartographer
