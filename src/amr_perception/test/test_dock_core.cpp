// Finding a V shaped dock, and refusing to find one that is not there.
//
// The refusals are the feature. Every rack leg, pallet corner and doorway in a
// warehouse is two surfaces meeting at an angle, so a detector that reports the
// first corner it sees will dock the vehicle to the building.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_perception/dock_core.hpp"

using amr_perception::DockSpec;
using amr_perception::findDock;
using amr_perception::fitLine;
using amr_perception::Point2;

namespace
{

/// A synthetic V, sampled the way a scanner would see it.
///
/// The apex sits at (ax, ay) with the opening facing the sensor, each face
/// `len` long, and `half` is half the opening angle. Noise is added so the
/// tests exercise the fit rather than an exact solution.
std::vector<Point2> makeV(
  double ax, double ay, double yaw, double len = 0.35,
  double half = M_PI / 4.0, double noise = 0.0, int per_face = 60)
{
  std::vector<Point2> pts;
  for (int side = -1; side <= 1; side += 2) {
    const double a = yaw + side * half;
    for (int i = 1; i <= per_face; ++i) {
      const double s = len * static_cast<double>(i) / per_face;
      // A deterministic wobble, so the test does not depend on a seed.
      const double w = noise * std::sin(static_cast<double>(i) * 1.7);
      pts.push_back({ax + s * std::cos(a) + w * std::sin(a),
                     ay + s * std::sin(a) - w * std::cos(a)});
    }
  }
  return pts;
}

}  // namespace

TEST(FitLine, RecoversADirectionAndReportsAResidual)
{
  std::vector<Point2> pts;
  for (int i = 0; i < 50; ++i) {
    const double s = 0.01 * i;
    pts.push_back({1.0 + s, 2.0 + s});      // 45 degrees
  }
  const auto line = fitLine(pts);
  ASSERT_TRUE(line.has_value());
  EXPECT_NEAR(std::abs(std::atan2(line->dy, line->dx)), M_PI / 4.0, 1e-9);
  EXPECT_LT(line->residual, 1e-9);
}

TEST(FitLine, HandlesAVerticalRunWhichASlopeFitCannot)
{
  // y = mx + c diverges here. Total least squares does not, and a dock face
  // seen square on is exactly this case.
  std::vector<Point2> pts;
  for (int i = 0; i < 40; ++i) {
    pts.push_back({1.0, 0.01 * i});
  }
  const auto line = fitLine(pts);
  ASSERT_TRUE(line.has_value());
  EXPECT_NEAR(std::abs(line->dx), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(line->dy), 1.0, 1e-9);
}

TEST(FitLine, RefusesFewerThanTwoPoints)
{
  EXPECT_FALSE(fitLine({}).has_value());
  EXPECT_FALSE(fitLine({{0.0, 0.0}}).has_value());
}

TEST(FindDock, LocatesACleanVToWithinAMillimetre)
{
  // Straight ahead at one metre, which is a plausible standoff.
  const auto pose = findDock(makeV(1.0, 0.0, M_PI));
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.0, 0.001);
  EXPECT_NEAR(pose->y, 0.0, 0.001);
  EXPECT_NEAR(pose->opening, M_PI / 2.0, 0.02);
}

TEST(FindDock, LocatesAnOffCentreV)
{
  // The case a flat plate cannot solve: displaced sideways, which changes no
  // range reading on a flat face and is fully observable on a V.
  const auto pose = findDock(makeV(1.0, 0.25, M_PI));
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.0, 0.005);
  EXPECT_NEAR(pose->y, 0.25, 0.005);
}

TEST(FindDock, SurvivesRealisticNoise)
{
  // 5 mm of wobble, well above this scanner's beam spacing at a metre.
  const auto pose = findDock(makeV(1.2, -0.10, M_PI, 0.35, M_PI / 4.0, 0.005));
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.2, 0.02);
  EXPECT_NEAR(pose->y, -0.10, 0.02);
}

TEST(FindDock, RefusesAFlatWall)
{
  // The single most likely false positive: a straight face fills the window.
  std::vector<Point2> wall;
  for (int i = -80; i <= 80; ++i) {
    wall.push_back({1.0, 0.01 * i});
  }
  EXPECT_FALSE(findDock(wall).has_value())
    << "a flat wall was reported as a dock, so the vehicle would dock to the building";
}

TEST(FindDock, RefusesTheWrongAngle)
{
  // A 150 degree corner is a wall with a kink, not a dock. Rack ends and
  // doorway reveals produce these constantly.
  const auto shallow = findDock(makeV(1.0, 0.0, M_PI, 0.35, M_PI * 15.0 / 180.0));
  EXPECT_FALSE(shallow.has_value());
}

TEST(FindDock, RefusesSomethingTooSmallToBeADock)
{
  // Correct angle, 30 mm of face: a pallet corner or a rack leg.
  const auto tiny = findDock(makeV(1.0, 0.0, M_PI, 0.03, M_PI / 4.0, 0.0, 20));
  EXPECT_FALSE(tiny.has_value())
    << "a corner the size of a rack leg passed as a dock";
}

TEST(FindDock, RefusesACurve)
{
  // A drum, a bin, a person's leg. The residual gate is what rejects it.
  std::vector<Point2> arc;
  for (int i = -60; i <= 60; ++i) {
    const double a = 0.012 * i;
    arc.push_back({1.0 + 0.25 * std::cos(a) - 0.25, 0.25 * std::sin(a)});
  }
  EXPECT_FALSE(findDock(arc).has_value());
}

TEST(FindDock, RefusesAnEmptyOrSparseScan)
{
  EXPECT_FALSE(findDock({}).has_value());
  EXPECT_FALSE(findDock(makeV(1.0, 0.0, M_PI, 0.35, M_PI / 4.0, 0.0, 3)).has_value());
}

TEST(FindDock, IgnoresAnythingOutsideTheWindow)
{
  // A real dock plus a wall behind it and clutter to the side. Only the dock
  // is inside the range and sector window, and the answer must not move.
  auto pts = makeV(1.0, 0.0, M_PI);
  for (int i = -100; i <= 100; ++i) {
    pts.push_back({4.0, 0.02 * i});             // wall well beyond max_range
    pts.push_back({0.3 * std::cos(1.2), 0.3 * std::sin(1.2)});   // outside the sector
  }
  const auto pose = findDock(pts);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.0, 0.005);
  EXPECT_NEAR(pose->y, 0.0, 0.005);
}

TEST(FindDock, TheHeadingPointsFromTheApexTowardTheSensor)
{
  // Which is the direction a vehicle approaches along, and getting the sign
  // wrong would send the controller through the dock.
  const auto pose = findDock(makeV(1.0, 0.0, M_PI));
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(std::abs(pose->yaw), M_PI, 0.02);
}
