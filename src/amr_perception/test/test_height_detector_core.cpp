// Unit tests for depth-based people detection.
//
// Depth images are synthesised by rendering known solids into a pinhole camera,
// so every case has exact truth. The two that matter are a person and a rack
// upright at the same place: the leg detector cannot tell those apart and the
// tracker cannot either unless the person is walking. This is the channel that
// can.

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "amr_perception/height_detector_core.hpp"

using amr_perception::HeightCluster;
using amr_perception::HeightParams;
using amr_perception::Intrinsics;
using amr_perception::Transform3D;
using amr_perception::clusterDepth;
using amr_perception::looksLikePerson;
using amr_perception::looksLikeStructure;

namespace
{

Intrinsics makeIntrinsics()
{
  // 640 x 480 at the D435's 87 degree horizontal field of view.
  Intrinsics k;
  k.width = 640;
  k.height = 480;
  k.cx = 319.5;
  k.cy = 239.5;
  k.fx = (640.0 / 2.0) / std::tan(87.0 * M_PI / 180.0 / 2.0);
  k.fy = (480.0 / 2.0) / std::tan(58.0 * M_PI / 180.0 / 2.0);
  return k;
}

/// Camera mounted at 0.27 m looking along robot +x, optical convention.
Transform3D makeExtrinsics(double height = 0.27)
{
  Transform3D tf;
  // optical (x right, y down, z forward) -> robot (x forward, y left, z up)
  tf.R = {0, 0, 1,
    -1, 0, 0,
    0, -1, 0};
  tf.t = {0.0, 0.0, height};
  return tf;
}

/// Render an axis-aligned vertical box in the ROBOT frame into a depth image.
/// Only the nearest surface per pixel is kept, which is what a depth sensor
/// gives.
void renderBox(
  std::vector<float> & depth, const Intrinsics & k, const Transform3D & tf,
  double cx, double cy, double half_x, double half_y, double z_lo, double z_hi)
{
  // Invert the extrinsics: robot -> optical, so a pixel ray can be walked.
  // The rotation is orthonormal, so the transpose is the inverse.
  const auto & R = tf.R;
  const auto & t = tf.t;

  for (std::size_t v = 0; v < k.height; ++v) {
    for (std::size_t u = 0; u < k.width; ++u) {
      const double dx = (static_cast<double>(u) - k.cx) / k.fx;
      const double dy = (static_cast<double>(v) - k.cy) / k.fy;
      // Ray direction in optical coords, unit z.
      const double ox = dx, oy = dy, oz = 1.0;
      // Rotate into robot frame (origin at the camera).
      const double rx = R[0] * ox + R[1] * oy + R[2] * oz;
      const double ry = R[3] * ox + R[4] * oy + R[5] * oz;
      const double rz = R[6] * ox + R[7] * oy + R[8] * oz;

      // March the ray. A closed-form slab intersection would be faster, but
      // this is a test and marching is obviously correct.
      const double step = 0.01;
      for (double s = 0.2; s < 8.0; s += step) {
        const double px = t[0] + rx * s;
        const double py = t[1] + ry * s;
        const double pz = t[2] + rz * s;
        if (pz < z_lo || pz > z_hi) {continue;}
        if (std::abs(px - cx) > half_x || std::abs(py - cy) > half_y) {continue;}
        const std::size_t idx = v * k.width + u;
        const float d = static_cast<float>(s);
        if (!std::isfinite(depth[idx]) || d < depth[idx]) {depth[idx] = d;}
        break;
      }
    }
  }
}

std::vector<float> emptyDepth(const Intrinsics & k)
{
  return std::vector<float>(k.width * k.height, std::numeric_limits<float>::quiet_NaN());
}

/// A person: narrow legs, wider torso, topping out at 1.75 m.
void renderPerson(std::vector<float> & d, const Intrinsics & k, const Transform3D & tf,
  double x, double y)
{
  renderBox(d, k, tf, x, y, 0.06, 0.14, 0.00, 0.84);   // legs
  renderBox(d, k, tf, x, y, 0.12, 0.23, 0.84, 1.46);   // torso
  renderBox(d, k, tf, x, y, 0.11, 0.11, 1.46, 1.75);   // head
}

/// A rack upright: same width all the way to the ceiling.
void renderUpright(std::vector<float> & d, const Intrinsics & k, const Transform3D & tf,
  double x, double y)
{
  renderBox(d, k, tf, x, y, 0.06, 0.06, 0.00, 3.00);
}

}  // namespace

TEST(HeightDetector, EmptyDepthYieldsNoClusters)
{
  const auto k = makeIntrinsics();
  HeightParams p;
  EXPECT_TRUE(clusterDepth(emptyDepth(k), k, makeExtrinsics(), p).empty());
}

TEST(HeightDetector, FindsAPersonAndPutsThemInTheRightPlace)
{
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderPerson(d, k, tf, 2.5, 0.0);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_FALSE(clusters.empty());

  const HeightCluster * best = nullptr;
  for (const auto & c : clusters) {
    if (!best || c.points > best->points) {best = &c;}
  }
  ASSERT_NE(best, nullptr);
  // The camera sees the near face, so the centroid sits a little short.
  EXPECT_NEAR(best->cx, 2.5, 0.3);
  EXPECT_NEAR(best->cy, 0.0, 0.2);
  EXPECT_GT(best->max_z, 1.3);
  EXPECT_TRUE(looksLikePerson(*best, p));
}

TEST(HeightDetector, RejectsARackUpright)
{
  // The whole reason this channel exists. On a 150 mm scan plane this object is
  // indistinguishable from a calf.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderUpright(d, k, tf, 2.5, 0.0);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_FALSE(clusters.empty());
  for (const auto & c : clusters) {
    EXPECT_FALSE(looksLikePerson(c, p))
      << "a rack upright reaching 3 m was classified as a person, top at " << c.max_z;
  }
}

TEST(HeightDetector, CallsATallColumnStructureWhenItCanSeeTheTop)
{
  // Far enough away that the 58 degree vertical field of view reaches past a
  // person's head: the visible ceiling is 0.27 + r*tan(29 deg), so about 3.3 m
  // at 5.5 m range.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderUpright(d, k, tf, 5.0, 0.0);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_FALSE(clusters.empty());
  bool any_structure = false;
  for (const auto & c : clusters) {
    any_structure = any_structure || looksLikeStructure(c, p);
  }
  EXPECT_TRUE(any_structure);
}

TEST(HeightDetector, KnowsWhenItCannotSeeTheTopOfSomething)
{
  // The limitation that made the previous test fail before truncation was
  // modelled. At 2.5 m the camera sees no higher than 1.66 m, so a 3 m upright
  // and a 1.75 m person are cut off at exactly the same apparent height. The
  // detector has to KNOW that rather than conclude the upright is person sized.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderUpright(d, k, tf, 2.5, 0.0);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_FALSE(clusters.empty());
  for (const auto & c : clusters) {
    EXPECT_TRUE(c.truncated)
      << "a 3 m column at 2.5 m was not recognised as cut off; top " << c.max_z
      << " visible ceiling " << c.visible_ceiling;
    EXPECT_FALSE(looksLikeStructure(c, p))
      << "claimed structure from a height it never observed";
    // And it must still not be called a person. The width profile does that.
    EXPECT_FALSE(looksLikePerson(c, p));
  }
}

TEST(HeightDetector, VisibleCeilingGrowsWithRange)
{
  HeightParams p;
  EXPECT_LT(amr_perception::visibleCeiling(1.5, p), 1.75)
    << "a person at 1.5 m should be cut off by the vertical field of view";
  EXPECT_GT(amr_perception::visibleCeiling(4.0, p), 1.75)
    << "a person at 4 m should fit inside the vertical field of view";
}

TEST(HeightDetector, SeparatesAPersonStandingBesideAnUpright)
{
  // The realistic case and the one the earlier channels get wrong: a worker
  // standing at a rack. Both are stationary, both are leg-sized on the plane.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderPerson(d, k, tf, 3.0, 0.6);
  renderUpright(d, k, tf, 3.0, -0.6);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_GE(clusters.size(), 2u);

  int people = 0;
  for (const auto & c : clusters) {
    if (looksLikePerson(c, p)) {
      ++people;
      EXPECT_GT(c.cy, 0.0) << "the person was found on the upright's side";
    }
  }
  EXPECT_EQ(people, 1) << "expected exactly one person among " << clusters.size()
                       << " clusters";
}

TEST(HeightDetector, GroundIsRemoved)
{
  // A floor plane, if kept, forms one enormous cluster that swallows everything
  // standing on it.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderBox(d, k, tf, 3.0, 0.0, 3.0, 3.0, -0.02, 0.02);   // the floor

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  for (const auto & c : clusters) {
    EXPECT_FALSE(looksLikePerson(c, p)) << "the floor was classified as a person";
  }
}

TEST(HeightDetector, StrideReducesWorkWithoutLosingThePerson)
{
  // Stride is the CPU budget knob. It must cost points, not detections.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderPerson(d, k, tf, 2.5, 0.0);

  HeightParams fine;
  fine.stride = 1;
  HeightParams coarse;
  coarse.stride = 4;

  const auto a = clusterDepth(d, k, tf, fine);
  const auto b = clusterDepth(d, k, tf, coarse);

  auto anyPerson = [](const std::vector<HeightCluster> & cs, const HeightParams & p) {
      for (const auto & c : cs) {if (looksLikePerson(c, p)) {return true;}}
      return false;
    };
  EXPECT_TRUE(anyPerson(a, fine));
  EXPECT_TRUE(anyPerson(b, coarse));

  std::size_t na = 0, nb = 0;
  for (const auto & c : a) {na += c.points;}
  for (const auto & c : b) {nb += c.points;}
  EXPECT_LT(nb, na) << "stride did not reduce the point count";
}

TEST(HeightDetector, RejectsSomethingTooShortToBeAPerson)
{
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderBox(d, k, tf, 2.5, 0.0, 0.20, 0.20, 0.0, 0.55);   // a crate

  HeightParams p;
  for (const auto & c : clusterDepth(d, k, tf, p)) {
    EXPECT_FALSE(looksLikePerson(c, p)) << "a crate was classified as a person";
  }
}

TEST(HeightDetector, RejectsSomethingTooWideToBeAPerson)
{
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderBox(d, k, tf, 3.0, 0.0, 0.20, 1.20, 0.0, 1.60);   // a pallet stack

  HeightParams p;
  for (const auto & c : clusterDepth(d, k, tf, p)) {
    EXPECT_FALSE(looksLikePerson(c, p)) << "a wide stack was classified as a person";
  }
}

TEST(HeightDetector, ATorsolessColumnInTheRightHeightBandIsStillRejected)
{
  // A post cut off at 1.6 m tops out exactly where a person does. Height alone
  // would accept it; the width profile is what rejects it.
  const auto k = makeIntrinsics();
  const auto tf = makeExtrinsics();
  auto d = emptyDepth(k);
  renderBox(d, k, tf, 2.5, 0.0, 0.06, 0.06, 0.0, 1.60);

  HeightParams p;
  const auto clusters = clusterDepth(d, k, tf, p);
  ASSERT_FALSE(clusters.empty());
  for (const auto & c : clusters) {
    EXPECT_FALSE(looksLikePerson(c, p))
      << "a uniform post topping out at person height was accepted; "
      << "leg width " << c.leg_width << " torso width " << c.torso_width;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
