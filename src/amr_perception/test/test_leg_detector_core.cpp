// Unit tests for the geometric leg detector.
//
// Scans are synthesised here rather than recorded, so every case has an exact
// known truth. The pedestrian model this targets has 110 mm diameter legs
// 200 mm apart, and the merged scan runs at 0.17 degree resolution.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "amr_perception/leg_detector_core.hpp"

using amr_perception::Cluster;
using amr_perception::DetectorParams;
using amr_perception::looksLikeLeg;
using amr_perception::pairLegs;
using amr_perception::detectPeople;
using amr_perception::segmentScan;

namespace
{
constexpr double kAngleMin = -M_PI;
constexpr double kInc = 0.17 * M_PI / 180.0;   // the deployed resolution
constexpr std::size_t kBins = 2118;

/// Empty scan: everything out of range.
std::vector<float> emptyScan()
{
  return std::vector<float>(kBins, std::numeric_limits<float>::infinity());
}

/// Paint a circle of radius `rad` centred at (cx, cy) into a scan, as the
/// nearest surface a ray would strike.
void paintCircle(std::vector<float> & scan, double cx, double cy, double rad)
{
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const double a = kAngleMin + static_cast<double>(i) * kInc;
    const double ux = std::cos(a), uy = std::sin(a);
    // Ray-circle intersection along the unit direction.
    const double b = ux * cx + uy * cy;
    const double c = cx * cx + cy * cy - rad * rad;
    const double disc = b * b - c;
    if (disc < 0.0) {continue;}
    const double t = b - std::sqrt(disc);
    if (t <= 0.0) {continue;}
    if (!std::isfinite(scan[i]) || t < scan[i]) {scan[i] = static_cast<float>(t);}
  }
}

/// Paint a flat wall segment perpendicular to x at distance `d`.
void paintWall(std::vector<float> & scan, double d, double half_extent)
{
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const double a = kAngleMin + static_cast<double>(i) * kInc;
    const double ux = std::cos(a), uy = std::sin(a);
    if (ux <= 1e-6) {continue;}
    const double t = d / ux;
    if (std::abs(t * uy) > half_extent) {continue;}
    if (!std::isfinite(scan[i]) || t < scan[i]) {scan[i] = static_cast<float>(t);}
  }
}

std::vector<Cluster> legsOf(const std::vector<float> & scan, const DetectorParams & p)
{
  std::vector<Cluster> legs;
  for (const auto & c : segmentScan(scan, kAngleMin, kInc, p)) {
    if (looksLikeLeg(c, p)) {legs.push_back(c);}
  }
  return legs;
}
}  // namespace

TEST(SegmentScan, EmptyScanYieldsNoClusters)
{
  DetectorParams p;
  EXPECT_TRUE(segmentScan(emptyScan(), kAngleMin, kInc, p).empty());
}

TEST(SegmentScan, ALongHoleBreaksARun)
{
  // A wide gap is a real discontinuity and must split the cluster.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, 0.0, 0.055);
  const auto before = segmentScan(scan, kAngleMin, kInc, p).size();
  ASSERT_EQ(before, 1u);

  for (std::size_t i = 0; i < scan.size(); ++i) {
    const double a = kAngleMin + static_cast<double>(i) * kInc;
    if (std::abs(a) < 0.020) {scan[i] = std::numeric_limits<float>::infinity();}
  }
  EXPECT_GT(segmentScan(scan, kAngleMin, kInc, p).size(), before);
}

TEST(SegmentScan, AShortHoleWithAgreeingSidesIsBridged)
{
  // The measured near-field failure. Re-binning two offset scanners about the
  // robot centre skips bins, and a pedestrian at 1.28 m arrived as seven
  // fragments none of which could be classified. A two-bin hole whose
  // neighbours sit on the same surface is a binning artifact, not a gap.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 1.3, 0.0, 0.055);
  const auto whole = segmentScan(scan, kAngleMin, kInc, p);
  ASSERT_EQ(whole.size(), 1u);

  // Punch isolated single-bin holes right through it.
  std::size_t punched = 0;
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const double a = kAngleMin + static_cast<double>(i) * kInc;
    if (std::abs(a) < 0.030 && (i % 3 == 0)) {
      scan[i] = std::numeric_limits<float>::infinity();
      ++punched;
    }
  }
  ASSERT_GT(punched, 3u);
  const auto after = segmentScan(scan, kAngleMin, kInc, p);
  EXPECT_EQ(after.size(), 1u)
    << "a perforated cluster fragmented into " << after.size() << " pieces";
}

TEST(SegmentScan, TwoSeparatedObjectsBecomeTwoClusters)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, -0.10, 0.055);
  paintCircle(scan, 2.0, 0.10, 0.055);
  EXPECT_EQ(segmentScan(scan, kAngleMin, kInc, p).size(), 2u);
}

TEST(SegmentScan, AdaptiveThresholdKeepsADistantObjectWhole)
{
  // The reason the break threshold grows with range. At 8 m the spacing between
  // adjacent returns is over four times what it is at 2 m, and a fixed
  // threshold would shatter the far leg into fragments.
  DetectorParams p;
  auto near = emptyScan();
  paintCircle(near, 2.0, 0.0, 0.055);
  auto far = emptyScan();
  paintCircle(far, 8.0, 0.0, 0.055);

  EXPECT_EQ(segmentScan(near, kAngleMin, kInc, p).size(), 1u);
  EXPECT_EQ(segmentScan(far, kAngleMin, kInc, p).size(), 1u);
}

TEST(SegmentScan, ClusterCentroidLandsOnTheObject)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 3.0, 0.5, 0.055);
  const auto clusters = segmentScan(scan, kAngleMin, kInc, p);
  ASSERT_EQ(clusters.size(), 1u);
  // The centroid sits on the near surface, so it is offset towards the sensor
  // by roughly the radius. Everything downstream must expect that.
  EXPECT_NEAR(clusters[0].cx, 3.0 - 0.055, 0.06);
  EXPECT_NEAR(clusters[0].cy, 0.5, 0.06);
}

TEST(LooksLikeLeg, AcceptsALegSizedRoundCluster)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, 0.0, 0.055);
  const auto clusters = segmentScan(scan, kAngleMin, kInc, p);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_TRUE(looksLikeLeg(clusters[0], p));
}

TEST(LooksLikeLeg, RejectsAFlatWall)
{
  // The discriminator that stops every pallet edge becoming a pedestrian.
  DetectorParams p;
  auto scan = emptyScan();
  paintWall(scan, 2.0, 1.5);
  const auto clusters = segmentScan(scan, kAngleMin, kInc, p);
  ASSERT_FALSE(clusters.empty());
  for (const auto & c : clusters) {
    EXPECT_FALSE(looksLikeLeg(c, p)) << "a flat wall was classified as a leg";
  }
}

TEST(LooksLikeLeg, RejectsSomethingFarTooWide)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, 0.0, 0.45);      // a pillar, not a calf
  const auto clusters = segmentScan(scan, kAngleMin, kInc, p);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_FALSE(looksLikeLeg(clusters[0], p));
}

TEST(LooksLikeLeg, RejectsATinyFragment)
{
  DetectorParams p;
  Cluster c;
  c.points = 2;
  c.width = 0.01;
  c.depth = 0.005;
  EXPECT_FALSE(looksLikeLeg(c, p));
}

TEST(PairLegs, PairsTwoLegsIntoOnePersonAtTheMidpoint)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, -0.10, 0.055);
  paintCircle(scan, 2.0, 0.10, 0.055);

  const auto people = pairLegs(legsOf(scan, p), p);
  ASSERT_EQ(people.size(), 1u);
  EXPECT_TRUE(people[0].paired);
  EXPECT_NEAR(people[0].y, 0.0, 0.05);
  EXPECT_NEAR(people[0].separation, 0.20, 0.06);
  EXPECT_GT(people[0].confidence, 0.5);
}

TEST(PairLegs, TwoPeopleStayTwoPeople)
{
  DetectorParams p;
  auto scan = emptyScan();
  // Two pedestrians two metres apart, each with a normal stance.
  paintCircle(scan, 3.0, -1.10, 0.055);
  paintCircle(scan, 3.0, -0.90, 0.055);
  paintCircle(scan, 3.0, 0.90, 0.055);
  paintCircle(scan, 3.0, 1.10, 0.055);

  const auto people = pairLegs(legsOf(scan, p), p);
  ASSERT_EQ(people.size(), 2u);
  for (const auto & q : people) {EXPECT_TRUE(q.paired);}
}

TEST(PairLegs, LegsTooFarApartAreNotOnePerson)
{
  // Otherwise two people standing a metre apart would merge into one phantom
  // pedestrian halfway between them, exactly where nobody is.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.0, -0.60, 0.055);
  paintCircle(scan, 2.0, 0.60, 0.055);

  const auto people = pairLegs(legsOf(scan, p), p);
  for (const auto & q : people) {
    EXPECT_FALSE(q.paired) << "legs 1.2 m apart were paired into one person";
  }
}

TEST(PairLegs, ASingleNearLegIsReportedWithLowerConfidence)
{
  // One leg occluding the other is common. Better a low-confidence detection
  // than nothing, but only close in.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 1.5, 0.0, 0.055);

  const auto people = pairLegs(legsOf(scan, p), p);
  ASSERT_EQ(people.size(), 1u);
  EXPECT_FALSE(people[0].paired);
  EXPECT_LT(people[0].confidence, 0.5);
}

TEST(PairLegs, ASingleDistantLegIsNotReported)
{
  // At long range a leg is a handful of points and clutter looks the same.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 9.0, 0.0, 0.055);

  EXPECT_TRUE(pairLegs(legsOf(scan, p), p).empty());
}

TEST(PairLegs, AWallProducesNoPeople)
{
  DetectorParams p;
  auto scan = emptyScan();
  paintWall(scan, 3.0, 4.0);
  EXPECT_TRUE(pairLegs(legsOf(scan, p), p).empty());
}

TEST(PairLegs, APersonInFrontOfAWallIsStillFound)
{
  // The realistic case. A pedestrian standing near warehouse racking must not
  // be swallowed by the structure behind them.
  DetectorParams p;
  auto scan = emptyScan();
  paintWall(scan, 4.0, 4.0);
  paintCircle(scan, 2.5, -0.10, 0.055);
  paintCircle(scan, 2.5, 0.10, 0.055);

  const auto people = pairLegs(legsOf(scan, p), p);
  ASSERT_EQ(people.size(), 1u);
  EXPECT_TRUE(people[0].paired);
  EXPECT_NEAR(people[0].x, 2.5 - 0.055, 0.10);
}

// ---- close range whole-body detection -------------------------------------
//
// The measured hole: at 1.28 m detection fell to 55 percent because the two
// calves and the gap between them arrive as one run too wide for the leg test.
// That is the worst possible range at which to lose a pedestrian.

TEST(DetectPeople, FindsACloseRangePersonWhoseLegsHaveMerged)
{
  DetectorParams p;
  auto scan = emptyScan();
  // A pedestrian at 1.2 m. At this range the angular gap between the calves is
  // small enough that the returns form a single cluster.
  paintCircle(scan, 1.2, -0.10, 0.055);
  paintCircle(scan, 1.2, 0.10, 0.055);

  const auto clusters = segmentScan(scan, kAngleMin, kInc, p);
  const auto people = detectPeople(clusters, p);
  ASSERT_FALSE(people.empty()) << clusters.size() << " clusters yielded nobody at 1.2 m";
  EXPECT_NEAR(people[0].x, 1.2 - 0.055, 0.15);
  EXPECT_NEAR(people[0].y, 0.0, 0.15);
}

TEST(DetectPeople, StillPairsLegsWhenTheyResolveSeparately)
{
  // The close-range path must not cannibalise the normal one.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 2.5, -0.10, 0.055);
  paintCircle(scan, 2.5, 0.10, 0.055);

  const auto people = detectPeople(segmentScan(scan, kAngleMin, kInc, p), p);
  ASSERT_EQ(people.size(), 1u);
  EXPECT_TRUE(people[0].paired) << "a resolvable pair was reported as a blob";
}

TEST(DetectPeople, DoesNotAcceptAWideBlobFurtherAway)
{
  // Beyond close range a stance-width cluster is a pallet or a bin, and the
  // legs of a real person would have resolved separately anyway.
  DetectorParams p;
  auto scan = emptyScan();
  paintCircle(scan, 5.0, 0.0, 0.30);

  for (const auto & q : detectPeople(segmentScan(scan, kAngleMin, kInc, p), p)) {
    EXPECT_TRUE(q.paired) << "a wide distant blob was reported as a person";
  }
}

TEST(DetectPeople, DoesNotAcceptAFlatCloseSurface)
{
  // A pallet face 1.5 m away is stance width and close, so only the roundness
  // test stands between it and a false pedestrian.
  DetectorParams p;
  auto scan = emptyScan();
  paintWall(scan, 1.5, 0.30);

  EXPECT_TRUE(detectPeople(segmentScan(scan, kAngleMin, kInc, p), p).empty());
}

TEST(DetectPeople, ABlobIsLessConfidentThanAResolvedPair)
{
  // An earlier version of this test assumed a pedestrian at 1.2 m arrives as
  // one merged blob. Measured on the running system, that is not what happens:
  // the legs still resolve and pair. The blob path exists for legs that are
  // genuinely together, so the test exercises that directly rather than
  // assuming a range at which it triggers.
  DetectorParams p;
  auto blob = emptyScan();
  paintCircle(blob, 1.5, 0.0, 0.16);          // legs together, one wide body
  auto pair = emptyScan();
  paintCircle(pair, 2.5, -0.10, 0.055);
  paintCircle(pair, 2.5, 0.10, 0.055);

  const auto a = detectPeople(segmentScan(blob, kAngleMin, kInc, p), p);
  const auto b = detectPeople(segmentScan(pair, kAngleMin, kInc, p), p);
  ASSERT_FALSE(a.empty());
  ASSERT_FALSE(b.empty());
  EXPECT_FALSE(a[0].paired);
  EXPECT_TRUE(b[0].paired);
  EXPECT_LT(a[0].confidence, b[0].confidence);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
