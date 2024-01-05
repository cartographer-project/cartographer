#include "cartographer/mapping/internal/3d/rotation_parameterization.h"

#include "ceres/manifold_test_utils.h"
#include "gtest/gtest.h"

namespace cartographer::mapping {

template <typename T>
class RotationParameterizationTests : public ::testing::Test {};

using TestTypes =
    ::testing::Types<YawOnlyQuaternionManifold, ConstantYawQuaternionManifold>;
TYPED_TEST_SUITE(RotationParameterizationTests, TestTypes);

TYPED_TEST(RotationParameterizationTests, ManifoldInvariantsHold) {
  const TypeParam manifold;

  constexpr static int kNumTrials = 10;
  constexpr static double kTolerance = 1.e-5;
  const std::vector<double> delta_magnitutes = {0.0, 1.e-9, 1.e-3, 0.5};
  for (int trial = 0; trial < kNumTrials; ++trial) {
    const Eigen::VectorXd x =
        Eigen::VectorXd::Random(manifold.AmbientSize()).normalized();

    for (const double delta_magnitude : delta_magnitutes) {
      const Eigen::VectorXd delta =
          Eigen::VectorXd::Random(manifold.TangentSize()) * delta_magnitude;
      EXPECT_THAT(manifold, ceres::XPlusZeroIsXAt(x, kTolerance));
      EXPECT_THAT(manifold, ceres::XMinusXIsZeroAt(x, kTolerance));
      EXPECT_THAT(manifold, ceres::MinusPlusIsIdentityAt(x, delta, kTolerance));
      const Eigen::VectorXd zero_tangent =
          Eigen::VectorXd::Zero(manifold.TangentSize());
      EXPECT_THAT(manifold,
                  ceres::MinusPlusIsIdentityAt(x, zero_tangent, kTolerance));

      Eigen::VectorXd y(manifold.AmbientSize());
      ASSERT_TRUE(manifold.Plus(x.data(), delta.data(), y.data()));
      EXPECT_THAT(manifold, ceres::PlusMinusIsIdentityAt(x, x, kTolerance));
      EXPECT_THAT(manifold, ceres::PlusMinusIsIdentityAt(x, y, kTolerance));
      EXPECT_THAT(manifold, ceres::HasCorrectPlusJacobianAt(x, kTolerance));
      EXPECT_THAT(manifold, ceres::HasCorrectMinusJacobianAt(x, kTolerance));
      EXPECT_THAT(manifold,
                  ceres::MinusPlusJacobianIsIdentityAt(x, kTolerance));
    }
  }
}

}  // namespace cartographer::mapping
