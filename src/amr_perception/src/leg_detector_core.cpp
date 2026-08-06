#include "amr_perception/leg_detector_core.hpp"

namespace amr_perception
{

namespace
{

struct Point
{
  double x;
  double y;
};

inline Point toPoint(double range, double angle)
{
  return {range * std::cos(angle), range * std::sin(angle)};
}

/// Straight-line distance from `p` to the segment ab.
double distanceToChord(const Point & a, const Point & b, const Point & p)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

}  // namespace

std::vector<Cluster> segmentScan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  const DetectorParams & p)
{
  std::vector<Cluster> clusters;
  std::vector<Point> run;
  std::size_t run_start = 0;
  bool have_prev = false;
  Point prev{0.0, 0.0};
  double prev_range = 0.0;

  auto flush = [&](std::size_t last_index) {
      if (run.size() < p.min_points) {
        run.clear();
        return;
      }
      Cluster c;
      c.first = run_start;
      c.last = last_index;
      c.points = run.size();

      double sx = 0.0, sy = 0.0;
      for (const auto & q : run) {sx += q.x; sy += q.y;}
      c.cx = sx / static_cast<double>(run.size());
      c.cy = sy / static_cast<double>(run.size());
      c.range = std::hypot(c.cx, c.cy);

      const Point & a = run.front();
      const Point & b = run.back();
      c.width = std::hypot(b.x - a.x, b.y - a.y);

      double bow = 0.0;
      for (const auto & q : run) {
        bow = std::max(bow, distanceToChord(a, b, q));
      }
      c.depth = bow;

      clusters.push_back(c);
      run.clear();
    };

  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r)) {
      // A hole in the data ends the run. Bridging it would invent continuity
      // that was never measured.
      if (have_prev) {flush(i > 0 ? i - 1 : 0);}
      have_prev = false;
      continue;
    }

    const Point pt = toPoint(r, angle_min + static_cast<double>(i) * angle_increment);

    if (have_prev) {
      // Adaptive threshold: point spacing grows with range because the sampling
      // is angular. A fixed threshold would shatter distant objects and glue
      // near ones together.
      const double allowed =
        p.cluster_jump_base + p.cluster_jump_slope * std::min(r, prev_range) * angle_increment;
      if (std::hypot(pt.x - prev.x, pt.y - prev.y) > allowed) {
        flush(i - 1);
        run_start = i;
      }
    } else {
      run_start = i;
    }

    run.push_back(pt);
    prev = pt;
    prev_range = r;
    have_prev = true;
  }

  if (have_prev) {flush(ranges.empty() ? 0 : ranges.size() - 1);}
  return clusters;
}

bool looksLikeLeg(const Cluster & c, const DetectorParams & p)
{
  if (c.points < p.min_points) {return false;}
  if (c.width < p.leg_width_min || c.width > p.leg_width_max) {return false;}
  // Roundness. A leg bows away from its chord; a wall segment of the same
  // width is flat. Expressed as a ratio so the test does not depend on how
  // wide the cluster happens to be.
  if (c.width > 1e-6 && (c.depth / c.width) < p.leg_min_depth_ratio) {return false;}
  return true;
}

std::vector<PersonDetection> pairLegs(
  const std::vector<Cluster> & legs, const DetectorParams & p)
{
  std::vector<PersonDetection> people;
  std::vector<bool> used(legs.size(), false);

  // Greedy nearest pairing. Legs are short and stance is narrow, so the nearest
  // admissible partner is the right one in all but contrived cases; a full
  // assignment here would be machinery without a payoff.
  for (std::size_t i = 0; i < legs.size(); ++i) {
    if (used[i]) {continue;}
    std::size_t best = legs.size();
    double best_d = p.pair_separation_max;

    for (std::size_t j = i + 1; j < legs.size(); ++j) {
      if (used[j]) {continue;}
      const double d = std::hypot(legs[i].cx - legs[j].cx, legs[i].cy - legs[j].cy);
      if (d >= p.pair_separation_min && d <= best_d) {
        best_d = d;
        best = j;
      }
    }

    if (best < legs.size()) {
      used[i] = used[best] = true;
      PersonDetection person;
      person.x = 0.5 * (legs[i].cx + legs[best].cx);
      person.y = 0.5 * (legs[i].cy + legs[best].cy);
      person.separation = best_d;
      person.paired = true;
      person.confidence = 0.9;
      people.push_back(person);
    } else if (legs[i].range <= p.single_leg_max_range) {
      // One leg occluding the other, or legs together. Reported, but with lower
      // confidence and only close in, where a single small cluster is unlikely
      // to be clutter.
      used[i] = true;
      PersonDetection person;
      person.x = legs[i].cx;
      person.y = legs[i].cy;
      person.paired = false;
      person.confidence = 0.4;
      people.push_back(person);
    }
  }

  return people;
}

}  // namespace amr_perception
