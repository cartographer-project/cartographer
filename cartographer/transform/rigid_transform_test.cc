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

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"

#include <random>

#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

template <typename T>
class RigidTransformTest : public ::testing::Test {
 protected:
  RigidTransformTest() {}

  T eps() {
    return std::numeric_limits<T>::epsilon();
  }

  Rigid2<T> GetRandomRigid2() {
    const T x = T(0.7) * distribution_(prng_);
    const T y = T(0.7) * distribution_(prng_);
    const T theta = T(0.2) * distribution_(prng_);
    return transform::Rigid2<T>(typename Rigid2<T>::Vector(x, y), theta);
  }

  Rigid3<T> GetRandomRigid3() {
    const T x = T(0.7) * distribution_(prng_);
    const T y = T(0.7) * distribution_(prng_);
    const T z = T(0.7) * distribution_(prng_);
    const T ax = T(0.7) * distribution_(prng_);
    const T ay = T(0.7) * distribution_(prng_);
    const T az = T(0.7) * distribution_(prng_);
    const T theta = T(0.2) * distribution_(prng_);
    return transform::Rigid3<T>(
        typename Rigid3<T>::Vector(x, y, z),
        typename Rigid3<T>::AngleAxis(
            theta, typename Rigid3<T>::Vector(ax, ay, az).normalized()));
  }

  std::mt19937 prng_ = std::mt19937(42);
  std::uniform_real_distribution<T> distribution_ =
      std::uniform_real_distribution<T>(-1., 1.);
};

typedef ::testing::Types<float, double> ScalarTypes;
TYPED_TEST_CASE(RigidTransformTest, ScalarTypes);

TYPED_TEST(RigidTransformTest, identity2DTest) {
  const auto pose = TestFixture::GetRandomRigid2();
  EXPECT_THAT(pose * Rigid2<TypeParam>(), IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(Rigid2<TypeParam>() * pose, IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(pose * Rigid2<TypeParam>::Identity(),
              IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(Rigid2<TypeParam>::Identity() * pose,
              IsNearly(pose, TestFixture::eps()));
}

TYPED_TEST(RigidTransformTest, inverse2DTest) {
  const auto pose = TestFixture::GetRandomRigid2();
  EXPECT_THAT(pose.inverse() * pose,
              IsNearly(Rigid2<TypeParam>::Identity(), TestFixture::eps()));
  EXPECT_THAT(pose * pose.inverse(),
              IsNearly(Rigid2<TypeParam>::Identity(), TestFixture::eps()));
}

TYPED_TEST(RigidTransformTest, identity3DTest) {
  const auto pose = TestFixture::GetRandomRigid3();
  EXPECT_THAT(pose * Rigid3<TypeParam>(), IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(Rigid3<TypeParam>() * pose, IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(pose * Rigid3<TypeParam>::Identity(),
              IsNearly(pose, TestFixture::eps()));
  EXPECT_THAT(Rigid3<TypeParam>::Identity() * pose,
              IsNearly(pose, TestFixture::eps()));
}

TYPED_TEST(RigidTransformTest, inverse3DTest) {
  const auto pose = TestFixture::GetRandomRigid3();
  EXPECT_THAT(pose.inverse() * pose,
              IsNearly(Rigid3<TypeParam>::Identity(), TestFixture::eps()));
  EXPECT_THAT(pose * pose.inverse(),
              IsNearly(Rigid3<TypeParam>::Identity(), TestFixture::eps()));
}

}  // namespace
}  // namespace transform
}  // namespace cartographer
