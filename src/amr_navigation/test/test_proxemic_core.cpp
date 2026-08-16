// The cost a planner pays for passing close to a person.
//
// Pure arithmetic, checked without a costmap, because the shape is the part
// worth arguing about and a nav2 layer cannot be instantiated without a
// lifecycle node, a tf buffer and a master grid.

#include <gtest/gtest.h>

#include "amr_navigation/proxemic_core.hpp"

using amr_navigation::kMaxProxemicCost;
using amr_navigation::proxemicCost;

namespace
{
constexpr double kRadius = 1.2;
constexpr double kPeak = 200.0;
}  // namespace

TEST(ProxemicCost, IsPeakAtThePersonsOwnPosition)
{
  EXPECT_DOUBLE_EQ(proxemicCost(0.0, kRadius, kPeak), kPeak);
}

TEST(ProxemicCost, ReachesExactlyZeroAtTheRadius)
{
  // The whole reason this is not a Gaussian. A cost that never quite reaches
  // zero puts a contribution from every person into every cell of the costmap
  // and degenerates the planner's tie breaking, and it has no natural radius,
  // so the tuning parameter becomes a standard deviation nobody can tie to a
  // distance a person would recognise.
  EXPECT_DOUBLE_EQ(proxemicCost(kRadius, kRadius, kPeak), 0.0);
  EXPECT_DOUBLE_EQ(proxemicCost(kRadius + 1e-9, kRadius, kPeak), 0.0);
  EXPECT_DOUBLE_EQ(proxemicCost(50.0, kRadius, kPeak), 0.0);
}

TEST(ProxemicCost, DecreasesMonotonicallyWithDistance)
{
  // The planner needs this. A cost that rises anywhere as you move away would
  // give it a reason to move TOWARD somebody.
  double previous = proxemicCost(0.0, kRadius, kPeak);
  for (double d = 0.01; d < kRadius; d += 0.01) {
    const double c = proxemicCost(d, kRadius, kPeak);
    EXPECT_LE(c, previous) << "cost rose between " << (d - 0.01) << " and " << d;
    previous = c;
  }
}

TEST(ProxemicCost, IsSoftNearTheEdgeAndSteepNearTheCentre)
{
  // The square, stated as behaviour rather than as a formula. A person merely
  // being nearby should barely register; the vehicle being almost on top of
  // one should dominate.
  const double near_edge = proxemicCost(0.9 * kRadius, kRadius, kPeak);
  const double near_centre = proxemicCost(0.1 * kRadius, kRadius, kPeak);
  EXPECT_LT(near_edge, 0.02 * kPeak);
  EXPECT_GT(near_centre, 0.75 * kPeak);
}

TEST(ProxemicCost, NeverReachesTheInscribedValue)
{
  // nav2_costmap_2d: LETHAL is 254, INSCRIBED is 253. Anything at or above the
  // inscribed value tells the planner the robot's BODY cannot be there, which
  // is a claim about geometry. This layer makes a claim about preference.
  //
  // It matters behaviourally: a person standing in a doorway would otherwise
  // close the only route and the vehicle would abort the mission rather than
  // wait, which is worse than passing them politely.
  EXPECT_LT(kMaxProxemicCost, 253);
  for (double d = 0.0; d < 2.0; d += 0.005) {
    EXPECT_LE(proxemicCost(d, kRadius, kMaxProxemicCost), kMaxProxemicCost);
  }
}

TEST(ProxemicCost, RefusesNonsenseRatherThanDividingByZero)
{
  EXPECT_DOUBLE_EQ(proxemicCost(0.5, 0.0, kPeak), 0.0);
  EXPECT_DOUBLE_EQ(proxemicCost(0.5, -1.0, kPeak), 0.0);
  // A negative distance is not "very close", it is a caller bug, and reading
  // it as very close would put peak cost somewhere arbitrary.
  EXPECT_DOUBLE_EQ(proxemicCost(-0.5, kRadius, kPeak), 0.0);
}

TEST(ProxemicCost, ScalesWithPeakAndNotWithRadius)
{
  // At the same FRACTION of the radius the cost is the same, so widening the
  // radius does not quietly make the vehicle more timid at a given distance
  // ratio; it changes how far the concern extends. The two parameters do
  // different jobs and this pins that.
  EXPECT_NEAR(
    proxemicCost(0.5 * 1.2, 1.2, kPeak),
    proxemicCost(0.5 * 2.4, 2.4, kPeak), 1e-9);
  EXPECT_NEAR(proxemicCost(0.3, kRadius, 100.0) * 2.0,
              proxemicCost(0.3, kRadius, 200.0), 1e-9);
}
