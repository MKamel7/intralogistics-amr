// Unit tests for the pedestrian tracker.
//
// Every scenario is synthesised with exact truth, so the assertions are about
// behaviour rather than about whatever a recording happened to contain.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "amr_perception/tracker_core.hpp"

using amr_perception::MultiObjectTracker;
using amr_perception::Observation;
using amr_perception::Track;
using amr_perception::TrackerParams;
using amr_perception::mahalanobisSquared;

namespace
{
constexpr double kDt = 1.0 / 14.28;   // the scanner's own rate

/// Feed a straight-line target for `n` steps and return the tracker.
void feedLine(
  MultiObjectTracker & tr, double x0, double y0, double vx, double vy,
  std::size_t n, double dt = kDt)
{
  for (std::size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i + 1) * dt;
    tr.update({Observation{x0 + vx * t, y0 + vy * t, 1.0}}, dt);
  }
}

const Track * findMoving(const MultiObjectTracker & tr)
{
  for (const auto & t : tr.tracks()) {
    if (t.isMoving(tr.params())) {return &t;}
  }
  return nullptr;
}
}  // namespace

TEST(Tracker, FirstObservationCreatesAnUnconfirmedTrack)
{
  MultiObjectTracker tr;
  tr.update({Observation{1.0, 2.0, 1.0}}, kDt);
  ASSERT_EQ(tr.tracks().size(), 1u);
  EXPECT_FALSE(tr.tracks()[0].confirmed);
  EXPECT_TRUE(tr.confirmedTracks().empty());
  EXPECT_NEAR(tr.tracks()[0].x[0], 1.0, 1e-9);
  EXPECT_NEAR(tr.tracks()[0].x[1], 2.0, 1e-9);
}

TEST(Tracker, ConfirmsAfterEnoughHits)
{
  MultiObjectTracker tr;
  feedLine(tr, 2.0, 0.0, 0.0, 0.0, 4);
  ASSERT_EQ(tr.tracks().size(), 1u);
  EXPECT_TRUE(tr.tracks()[0].confirmed);
  EXPECT_EQ(tr.confirmedTracks().size(), 1u);
}

TEST(Tracker, ASingleFrameOfClutterNeverConfirms)
{
  // The reason confirmation exists. One spurious detection must not become a
  // pedestrian that the vehicle then brakes for.
  MultiObjectTracker tr;
  tr.update({Observation{5.0, 5.0, 1.0}}, kDt);
  for (int i = 0; i < 8; ++i) {
    tr.update({}, kDt);
  }
  EXPECT_TRUE(tr.tracks().empty());
}

TEST(Tracker, EstimatesVelocityOfAWalkingTarget)
{
  MultiObjectTracker tr;
  feedLine(tr, 3.0, 0.0, -1.2, 0.0, 40);
  const Track * t = findMoving(tr);
  ASSERT_NE(t, nullptr);
  EXPECT_NEAR(t->x[2], -1.2, 0.20);
  EXPECT_NEAR(t->x[3], 0.0, 0.20);
  EXPECT_NEAR(t->speed(), 1.2, 0.20);
}

TEST(Tracker, AStationaryTargetIsNeverClassifiedAsMoving)
{
  // This is the property that separates a pedestrian from a rack upright, and
  // the ONLY thing tracking contributes to that separation. Confirmation does
  // not help: a post is seen in every frame and confirms immediately.
  MultiObjectTracker tr;
  feedLine(tr, 4.0, 1.0, 0.0, 0.0, 60);
  ASSERT_FALSE(tr.tracks().empty());
  for (const auto & t : tr.tracks()) {
    EXPECT_FALSE(t.isMoving(tr.params()))
      << "a stationary target was called moving at " << t.speed() << " m/s";
  }
}

TEST(Tracker, APostConfirmsJustAsReadilyAsAPerson)
{
  // Stated as a test so nobody later assumes confirmation filters structure.
  MultiObjectTracker tr;
  feedLine(tr, 4.0, 1.0, 0.0, 0.0, 10);
  ASSERT_EQ(tr.confirmedTracks().size(), 1u);
  EXPECT_FALSE(tr.confirmedTracks()[0].isMoving(tr.params()));
}

TEST(Tracker, KeepsTheSameIdAcrossManyFrames)
{
  MultiObjectTracker tr;
  feedLine(tr, 3.0, 0.0, -0.8, 0.0, 5);
  ASSERT_EQ(tr.tracks().size(), 1u);
  const auto id = tr.tracks()[0].id;
  feedLine(tr, 3.0 - 0.8 * 5 * kDt, 0.0, -0.8, 0.0, 30);
  ASSERT_EQ(tr.tracks().size(), 1u);
  EXPECT_EQ(tr.tracks()[0].id, id) << "track identity was not preserved";
}

TEST(Tracker, CoastsThroughAShortOcclusionAndKeepsItsId)
{
  // A person walking behind a rack upright. Losing the identity here would
  // show up as an ID switch in the metrics and as a fresh unconfirmed track in
  // the behaviour, meaning the vehicle briefly forgets someone exists.
  MultiObjectTracker tr;
  feedLine(tr, 4.0, 0.0, -1.0, 0.0, 20);
  const Track * before = findMoving(tr);
  ASSERT_NE(before, nullptr);
  const auto id = before->id;
  const double x_at_loss = before->x[0];

  for (int i = 0; i < 6; ++i) {
    tr.update({}, kDt);           // fully occluded
  }
  ASSERT_FALSE(tr.tracks().empty());

  // Reappears where constant velocity says it should be.
  const double x_after = x_at_loss - 1.0 * 6 * kDt;
  tr.update({Observation{x_after, 0.0, 1.0}}, kDt);

  bool found = false;
  for (const auto & t : tr.tracks()) {
    if (t.id == id) {
      found = true;
      EXPECT_EQ(t.misses, 0u);
    }
  }
  EXPECT_TRUE(found) << "identity lost across a six frame occlusion";
}

TEST(Tracker, DropsATrackAfterAProlongedDisappearance)
{
  MultiObjectTracker tr;
  feedLine(tr, 3.0, 0.0, 0.0, 0.0, 6);
  ASSERT_EQ(tr.tracks().size(), 1u);
  for (std::size_t i = 0; i < TrackerParams().max_misses + 2; ++i) {
    tr.update({}, kDt);
  }
  EXPECT_TRUE(tr.tracks().empty());
}

TEST(Tracker, TwoSeparatedTargetsGetSeparateIds)
{
  MultiObjectTracker tr;
  for (int i = 0; i < 10; ++i) {
    tr.update({Observation{3.0, -2.0, 1.0}, Observation{3.0, 2.0, 1.0}}, kDt);
  }
  ASSERT_EQ(tr.confirmedTracks().size(), 2u);
  EXPECT_NE(tr.confirmedTracks()[0].id, tr.confirmedTracks()[1].id);
}

TEST(Tracker, TwoTargetsCrossingKeepTheirIdentities)
{
  // The classic multi-object failure. Two people walking past each other must
  // not swap identities, or the predicted paths swap with them and the vehicle
  // yields to the wrong side.
  MultiObjectTracker tr;
  const double v = 1.0;
  // Well separated in y throughout, converging then diverging in x.
  for (int i = 0; i < 60; ++i) {
    const double t = static_cast<double>(i) * kDt;
    tr.update(
      {Observation{-3.0 + v * t, -0.9, 1.0},
        Observation{3.0 - v * t, 0.9, 1.0}}, kDt);
  }
  const auto tracks = tr.confirmedTracks();
  ASSERT_EQ(tracks.size(), 2u);
  // One is travelling in +x, the other in -x. If identities had swapped, both
  // would carry the wrong sign of velocity relative to their side.
  for (const auto & t : tracks) {
    if (t.x[1] < 0.0) {
      EXPECT_GT(t.x[2], 0.0) << "the target on the -y side lost its direction";
    } else {
      EXPECT_LT(t.x[2], 0.0) << "the target on the +y side lost its direction";
    }
  }
}

TEST(Tracker, GateRejectsAnImplausibleJump)
{
  MultiObjectTracker tr;
  feedLine(tr, 2.0, 0.0, 0.0, 0.0, 6);
  ASSERT_EQ(tr.tracks().size(), 1u);
  const auto id = tr.tracks()[0].id;

  // A detection twenty metres away cannot be the same object.
  tr.update({Observation{22.0, 0.0, 1.0}}, kDt);
  bool original_still_there = false;
  for (const auto & t : tr.tracks()) {
    if (t.id == id) {
      original_still_there = true;
      EXPECT_NEAR(t.x[0], 2.0, 0.5) << "the track was dragged to the outlier";
    }
  }
  EXPECT_TRUE(original_still_there);
  EXPECT_EQ(tr.tracks().size(), 2u) << "the outlier should start its own track";
}

TEST(Tracker, UncertaintyGrowsWhileCoasting)
{
  // Why the gate is Mahalanobis and not metric. A track that has not been seen
  // for half a second is far less sure where its target is, and must be allowed
  // to look further afield when it reappears.
  MultiObjectTracker tr;
  feedLine(tr, 3.0, 0.0, -1.0, 0.0, 10);
  ASSERT_FALSE(tr.tracks().empty());
  const Observation probe{tr.tracks()[0].x[0] + 0.8, tr.tracks()[0].x[1], 1.0};
  const double before = mahalanobisSquared(tr.tracks()[0], probe, 0.12);

  for (int i = 0; i < 8; ++i) {tr.update({}, kDt);}
  ASSERT_FALSE(tr.tracks().empty());
  const double after = mahalanobisSquared(tr.tracks()[0], probe, 0.12);

  EXPECT_LT(after, before)
    << "a coasting track did not widen its gate, so it cannot re-acquire";
}

TEST(Tracker, PredictMovesStateAlongItsVelocity)
{
  MultiObjectTracker tr;
  feedLine(tr, 5.0, 0.0, -1.5, 0.0, 30);
  const Track * t = findMoving(tr);
  ASSERT_NE(t, nullptr);
  const double x0 = t->x[0];
  const double vx = t->x[2];
  tr.predict(0.5);
  const Track * after = findMoving(tr);
  ASSERT_NE(after, nullptr);
  EXPECT_NEAR(after->x[0], x0 + vx * 0.5, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
