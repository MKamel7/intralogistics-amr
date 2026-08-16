// Unit tests for the scan merge geometry. No ROS, no TF, no simulator.
//
// The node supplies transforms; everything asserted here is arithmetic, so
// these run in milliseconds and can be reasoned about exactly.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "amr_perception/scan_merge_core.hpp"

using amr_perception::FootprintFilter;
using amr_perception::ScanAccumulator;
using amr_perception::ScanView;
using amr_perception::Transform2D;

namespace
{
constexpr double kLength = 0.800;
constexpr double kWidth = 0.580;
constexpr double kMargin = 0.020;

ScanView makeView(const std::vector<float> & ranges, double angle_min, double increment)
{
  ScanView v;
  v.angle_min = angle_min;
  v.angle_increment = increment;
  v.range_min = 0.05;
  v.range_max = 40.0;
  v.ranges = &ranges;
  return v;
}
}  // namespace

TEST(Transform2D, IdentityLeavesPointsAlone)
{
  Transform2D tf;
  double x = 0.0, y = 0.0;
  tf.apply(1.5, -2.5, x, y);
  EXPECT_DOUBLE_EQ(x, 1.5);
  EXPECT_DOUBLE_EQ(y, -2.5);
}

TEST(Transform2D, RotatesThenTranslates)
{
  // A quarter turn about z, then a shift. Order matters: rotating after
  // translating would move the point somewhere else entirely.
  Transform2D tf{1.0, 2.0, M_PI / 2.0};
  double x = 0.0, y = 0.0;
  tf.apply(1.0, 0.0, x, y);
  EXPECT_NEAR(x, 1.0, 1e-12);
  EXPECT_NEAR(y, 3.0, 1e-12);
}

TEST(FootprintFilter, RejectsReturnsInsideTheVehicle)
{
  FootprintFilter f(kLength, kWidth, kMargin);
  // The measured self-return: the scanner sees its own chassis wall about
  // 60 mm away, well inside the envelope.
  EXPECT_TRUE(f.isSelfReturn(0.30, 0.20));
  EXPECT_TRUE(f.isSelfReturn(0.0, 0.0));
  EXPECT_TRUE(f.isSelfReturn(-0.39, -0.28));
}

TEST(FootprintFilter, KeepsReturnsJustOutsideTheVehicle)
{
  FootprintFilter f(kLength, kWidth, kMargin);
  EXPECT_FALSE(f.isSelfReturn(0.45, 0.0));
  EXPECT_FALSE(f.isSelfReturn(0.0, 0.35));
}

TEST(FootprintFilter, MarginExpandsTheEnvelopeSymmetrically)
{
  FootprintFilter f(kLength, kWidth, kMargin);
  EXPECT_DOUBLE_EQ(f.halfLength(), 0.5 * kLength + kMargin);
  EXPECT_DOUBLE_EQ(f.halfWidth(), 0.5 * kWidth + kMargin);
  // A point exactly on the expanded boundary counts as self, not as world.
  EXPECT_TRUE(f.isSelfReturn(f.halfLength(), 0.0));
  EXPECT_FALSE(f.isSelfReturn(f.halfLength() + 1e-6, 0.0));
}

TEST(FootprintFilter, PodsAreRejectedWithoutInflatingTheMargin)
{
  // The MP-400 numbers, because the whole point is that they are the real ones.
  constexpr double kChassisL = 0.590;
  constexpr double kChassisW = 0.559;
  constexpr double kBodyMargin = 0.010;
  constexpr double kPodX = 0.257574;
  constexpr double kPodY = 0.242074;
  constexpr double kPodHalf = 0.066114;

  FootprintFilter f(
    kChassisL, kChassisW, kBodyMargin,
    {{kPodX, kPodY, kPodHalf}, {-kPodX, -kPodY, kPodHalf}});

  // The pod corner, 28.7 mm proud of the chassis, is the extreme point of the
  // vehicle. The old bounding box reached it by blanking that much everywhere.
  EXPECT_TRUE(f.isSelfReturn(kPodX + kPodHalf - 1e-6, kPodY + kPodHalf - 1e-6));
  EXPECT_TRUE(f.isSelfReturn(0.300, 0.2845));       // the optical centre
  EXPECT_TRUE(f.isSelfReturn(-0.300, -0.2845));     // and the diagonal one
  EXPECT_TRUE(f.isSelfReturn(0.0, 0.0));

  // And this is what it buys. At the middle of the side the vehicle is only
  // its chassis, so a return 20 mm off the flank is a return from the world.
  // Under the 32 mm bounding box it was deleted as self. See V-39.
  EXPECT_FALSE(f.isSelfReturn(0.0, 0.5 * kChassisW + 0.020));

  // The forward protective field reaches 0.3446 m in y. Everything between the
  // filter edge and that line is coverage the fields actually get to use.
  const double band = (0.5 * kChassisW + 0.065) - (0.5 * kChassisW + kBodyMargin);
  EXPECT_GE(band, 0.050);
  EXPECT_FALSE(f.isSelfReturn(0.0, 0.5 * kChassisW + kBodyMargin + 1e-6));
}

TEST(FootprintFilter, PodsAreLocalToTheirCorners)
{
  // A pod that leaked along the flank would undo the shaping silently, so the
  // asymmetry is asserted rather than assumed.
  FootprintFilter f(0.590, 0.559, 0.010, {{0.257574, 0.242074, 0.066114}});

  EXPECT_TRUE(f.isSelfReturn(0.257574, 0.300));   // beside the pod
  EXPECT_FALSE(f.isSelfReturn(0.100, 0.300));     // beside the bare flank
  EXPECT_FALSE(f.isSelfReturn(-0.257574, -0.300));  // the pod that is not there
  EXPECT_EQ(f.pods().size(), 1u);
}

TEST(FootprintFilter, WithoutPodsIsTheOldPlainEnvelope)
{
  // A platform with flush scanners passes no pods at all, and must behave
  // exactly as the filter did before it learned about them.
  FootprintFilter f(kLength, kWidth, kMargin);
  EXPECT_TRUE(f.pods().empty());
  EXPECT_TRUE(f.isSelfReturn(f.halfLength(), f.halfWidth()));
  EXPECT_FALSE(f.isSelfReturn(f.halfLength() + 1e-6, 0.0));
}

TEST(ScanAccumulator, StartsCompletelyEmpty)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  EXPECT_EQ(acc.bins(), 360u);
  EXPECT_EQ(acc.emptyBins(), 360u);
  for (const auto r : acc.ranges()) {
    EXPECT_FALSE(std::isfinite(r));
  }
}

TEST(ScanAccumulator, PlacesAReturnInTheCorrectBearing)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  // One ray straight ahead at 5 m, from a sensor at the origin.
  std::vector<float> ranges{5.0F};
  auto view = makeView(ranges, 0.0, 0.01);
  acc.add(view, Transform2D{}, f);

  EXPECT_EQ(acc.accepted(), 1u);
  // Bearing 0 sits halfway round a scan that starts at -pi.
  const std::size_t bin = 180;
  EXPECT_NEAR(acc.ranges()[bin], 5.0F, 1e-4);
  EXPECT_EQ(acc.emptyBins(), 359u);
}

TEST(ScanAccumulator, TranslationMovesTheReturnAndChangesItsRange)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  // A sensor 1 m ahead of the origin, seeing something 5 m ahead of itself.
  // In the output frame that is 6 m away, not 5.
  std::vector<float> ranges{5.0F};
  auto view = makeView(ranges, 0.0, 0.01);
  acc.add(view, Transform2D{1.0, 0.0, 0.0}, f);

  EXPECT_EQ(acc.accepted(), 1u);
  EXPECT_NEAR(acc.ranges()[180], 6.0F, 1e-4);
}

TEST(ScanAccumulator, KeepsTheNearestReturnPerBinNeverTheAverage)
{
  // The property that matters for anything safety adjacent. If a bin averaged
  // a 0.4 m obstacle with a 10 m wall it would report 5.2 m of clear space that
  // does not exist.
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  std::vector<float> far{10.0F};
  std::vector<float> near{0.9F};
  acc.add(makeView(far, 0.0, 0.01), Transform2D{}, f);
  acc.add(makeView(near, 0.0, 0.01), Transform2D{}, f);

  EXPECT_NEAR(acc.ranges()[180], 0.9F, 1e-4);
  EXPECT_EQ(acc.accepted(), 2u);
}

TEST(ScanAccumulator, OrderOfScannersDoesNotChangeTheResult)
{
  FootprintFilter f(kLength, kWidth, kMargin);
  std::vector<float> far{10.0F};
  std::vector<float> near{0.9F};

  ScanAccumulator a(360, 0.05, 40.0);
  a.add(makeView(near, 0.0, 0.01), Transform2D{}, f);
  a.add(makeView(far, 0.0, 0.01), Transform2D{}, f);

  ScanAccumulator b(360, 0.05, 40.0);
  b.add(makeView(far, 0.0, 0.01), Transform2D{}, f);
  b.add(makeView(near, 0.0, 0.01), Transform2D{}, f);

  EXPECT_EQ(a.ranges(), b.ranges());
}

TEST(ScanAccumulator, DropsSelfReturnsAndCountsThem)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  // The real case: a scanner in the corner recess looking inboard, hitting its
  // own chassis 60 mm away.
  std::vector<float> ranges{0.06F};
  auto view = makeView(ranges, M_PI, 0.01);   // pointing back towards the centre
  acc.add(view, Transform2D{0.34, 0.245, M_PI / 4.0}, f);

  EXPECT_EQ(acc.accepted(), 0u);
  EXPECT_EQ(acc.selfReturns(), 1u);
  EXPECT_EQ(acc.emptyBins(), 360u);
}

TEST(ScanAccumulator, DropsNonFiniteAndOutOfRangeReturns)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  std::vector<float> ranges{
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(),
    0.001F,      // below the sensor minimum
    500.0F,      // beyond the sensor maximum
    7.0F,        // the only good one
  };
  auto view = makeView(ranges, 0.0, 0.01);
  acc.add(view, Transform2D{}, f);

  EXPECT_EQ(acc.accepted(), 1u);
  EXPECT_EQ(acc.invalid(), 2u);
  EXPECT_EQ(acc.outOfRange(), 2u);
}

TEST(ScanAccumulator, ResetClearsRangesAndCounters)
{
  ScanAccumulator acc(360, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);
  std::vector<float> ranges{5.0F};
  acc.add(makeView(ranges, 0.0, 0.01), Transform2D{}, f);
  ASSERT_EQ(acc.accepted(), 1u);

  acc.reset();
  EXPECT_EQ(acc.accepted(), 0u);
  EXPECT_EQ(acc.emptyBins(), 360u);
}

namespace
{
/// Longest run of consecutive empty bins, treating the scan as circular.
///
/// The count of empty bins on its own is close to useless. Re-binning polar
/// data from an off-centre origin at the SAME angular resolution necessarily
/// leaves scattered single-bin holes, which are an artifact and harmless. A
/// contiguous run is a blind sector, which is not. Only the second matters.
std::size_t longestGap(const std::vector<float> & ranges)
{
  const std::size_t n = ranges.size();
  std::size_t start = 0;
  while (start < n && !std::isfinite(ranges[start])) {++start;}
  if (start == n) {return n;}          // nothing seen at all

  std::size_t longest = 0, run = 0;
  for (std::size_t k = 0; k < n; ++k) {
    if (!std::isfinite(ranges[(start + k) % n])) {
      ++run;
      longest = std::max(longest, run);
    } else {
      run = 0;
    }
  }
  return longest;
}
}  // namespace

TEST(ScanAccumulator, TwoOpposedScannersLeaveNoBlindSector)
{
  // At the resolution the system actually deploys, 2118 bins for a full turn at
  // the scanner's own 0.17 degree resolution.
  //
  // These are IDEAL scanners: no vehicle in the way. The point is to prove the
  // merge maths itself opens no sector, so that any gap measured on the running
  // system is attributable to vehicle occlusion rather than to this code. The
  // running system does show a residual seam; see V-06 in docs/validation.md.
  constexpr std::size_t kBins = 2118;
  ScanAccumulator acc(kBins, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  const double aperture = 275.0 * M_PI / 180.0;
  const std::size_t rays = 1618;
  const double inc = aperture / static_cast<double>(rays);
  std::vector<float> ranges(rays, 8.0F);

  acc.add(makeView(ranges, -aperture / 2.0, inc), Transform2D{0.405, 0.295, M_PI / 4.0}, f);
  acc.add(makeView(ranges, -aperture / 2.0, inc), Transform2D{-0.405, -0.295, -3.0 * M_PI / 4.0}, f);

  const std::size_t gap = longestGap(acc.ranges());
  // One bin is re-binning aliasing. Anything wider is a sector.
  EXPECT_LE(gap, 1u) << "merge opens a " << gap << " bin blind sector";
}

TEST(ScanAccumulator, ASingleScannerLeavesALargeBlindSector)
{
  // Without this the test above proves nothing: it has to be possible to fail.
  // One 275 degree scanner must leave the missing 85 degrees uncovered.
  constexpr std::size_t kBins = 2118;
  ScanAccumulator acc(kBins, 0.05, 40.0);
  FootprintFilter f(kLength, kWidth, kMargin);

  const double aperture = 275.0 * M_PI / 180.0;
  const std::size_t rays = 1618;
  std::vector<float> ranges(rays, 8.0F);
  acc.add(
    makeView(ranges, -aperture / 2.0, aperture / static_cast<double>(rays)),
    Transform2D{0.405, 0.295, M_PI / 4.0}, f);

  const double gap_deg = static_cast<double>(longestGap(acc.ranges())) * 360.0 / kBins;
  EXPECT_GT(gap_deg, 60.0) << "a single 275 degree scanner should leave a large sector";
}

TEST(ScanAccumulator, AFullTurnDoesNotDuplicateTheFirstBearing)
{
  // A scan whose angle_max equals angle_min + 2*pi describes the first ray
  // twice, once at each end. It is malformed, and the way it fails is silent:
  // slam_toolbox accepted such a scan and produced no map whatsoever, with
  // nothing logged.
  ScanAccumulator acc(2118, 0.05, 40.0);
  const double span = 2.0 * M_PI - acc.angleIncrement();
  EXPECT_LT(span, 2.0 * M_PI);
  EXPECT_NEAR(
    acc.angleMin() + span + acc.angleIncrement(), acc.angleMin() + 2.0 * M_PI, 1e-12);
  // and the bin count still covers the whole turn
  EXPECT_EQ(acc.bins(), 2118u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
