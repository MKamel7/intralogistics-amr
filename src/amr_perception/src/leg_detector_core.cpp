#include "amr_perception/leg_detector_core.hpp"

#include <map>
#include <utility>

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

std::vector<Cluster> clusterPoints(
  const std::vector<Point2> & points, const DetectorParams & p)
{
  const double cell = p.cluster_cell > 1e-6 ? p.cluster_cell : 0.06;
  std::map<std::pair<int, int>, std::vector<std::size_t>> grid;
  for (std::size_t i = 0; i < points.size(); ++i) {
    grid[{static_cast<int>(std::floor(points[i].x / cell)),
      static_cast<int>(std::floor(points[i].y / cell))}].push_back(i);
  }

  std::map<std::pair<int, int>, bool> seen;
  std::vector<Cluster> clusters;

  for (const auto & entry : grid) {
    if (seen[entry.first]) {continue;}
    std::vector<std::size_t> members;
    std::vector<std::pair<int, int>> stack{entry.first};
    seen[entry.first] = true;

    while (!stack.empty()) {
      const auto k = stack.back();
      stack.pop_back();
      auto it = grid.find(k);
      if (it == grid.end()) {continue;}
      members.insert(members.end(), it->second.begin(), it->second.end());
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {continue;}
          const std::pair<int, int> nb{k.first + dx, k.second + dy};
          if (grid.count(nb) && !seen[nb]) {
            seen[nb] = true;
            stack.push_back(nb);
          }
        }
      }
    }

    if (members.size() < p.min_points) {continue;}

    Cluster c;
    c.points = members.size();
    double sx = 0.0, sy = 0.0;
    for (const auto i : members) {sx += points[i].x; sy += points[i].y;}
    c.cx = sx / static_cast<double>(members.size());
    c.cy = sy / static_cast<double>(members.size());
    c.range = std::hypot(c.cx, c.cy);

    // Width is the widest separation in the cluster, and the chord it defines
    // is what depth is measured against. Clusters here are tens of points, so
    // the quadratic scan costs nothing and avoids assuming any ordering.
    std::size_t a = members.front(), b = members.front();
    double best = 0.0;
    for (std::size_t i = 0; i < members.size(); ++i) {
      for (std::size_t j = i + 1; j < members.size(); ++j) {
        const double d = std::hypot(
          points[members[i]].x - points[members[j]].x,
          points[members[i]].y - points[members[j]].y);
        if (d > best) {best = d; a = members[i]; b = members[j];}
      }
    }
    c.width = best;

    const Point pa{points[a].x, points[a].y};
    const Point pb{points[b].x, points[b].y};
    double bow = 0.0;
    for (const auto i : members) {
      bow = std::max(bow, distanceToChord(pa, pb, Point{points[i].x, points[i].y}));
    }
    c.depth = bow;

    clusters.push_back(c);
  }

  return clusters;
}

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

  std::size_t empty_run = 0;
  std::size_t last_filled = 0;

  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r)) {
      // Hold the run open across a short hole. Whether it survives is decided
      // below, by whether the point on the far side actually continues the
      // surface; see max_bridge_bins for why an empty bin here is usually a
      // re-binning artifact rather than an absence of measurement.
      if (have_prev) {
        ++empty_run;
        if (empty_run > p.max_bridge_bins) {
          flush(last_filled);
          have_prev = false;
          empty_run = 0;
        }
      }
      continue;
    }

    const Point pt = toPoint(r, angle_min + static_cast<double>(i) * angle_increment);

    if (have_prev) {
      // Adaptive threshold: point spacing grows with range because the sampling
      // is angular. A fixed threshold would shatter distant objects and glue
      // near ones together. Bridging a hole widens the allowance in proportion
      // to how many bins were skipped, so a wide hole demands closer agreement
      // relative to the distance covered rather than less.
      const double allowed =
        (p.cluster_jump_base
        + p.cluster_jump_slope * std::min(r, prev_range) * angle_increment)
        * static_cast<double>(empty_run + 1);
      if (std::hypot(pt.x - prev.x, pt.y - prev.y) > allowed) {
        flush(last_filled);
        run_start = i;
      }
    } else {
      run_start = i;
    }
    empty_run = 0;
    last_filled = i;

    run.push_back(pt);
    prev = pt;
    prev_range = r;
    have_prev = true;
  }

  if (have_prev) {flush(last_filled);}
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

bool looksLikePersonBlob(const Cluster & c, const DetectorParams & p)
{
  if (c.points < p.min_points) {return false;}
  // Only close in. Further out a cluster this wide is a pallet, a bin or a wall
  // section, and the legs would have resolved separately anyway.
  if (c.range > p.blob_max_range) {return false;}
  if (c.width < p.blob_width_min || c.width > p.blob_width_max) {return false;}
  // Still has to bow away from its chord. A flat run of stance width is a
  // pallet face, not a person.
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

std::vector<PersonDetection> detectPeople(
  const std::vector<Cluster> & clusters, const DetectorParams & p)
{
  std::vector<Cluster> legs;
  std::vector<Cluster> rest;
  for (const auto & c : clusters) {
    if (looksLikeLeg(c, p)) {
      legs.push_back(c);
    } else {
      rest.push_back(c);
    }
  }

  auto people = pairLegs(legs, p);

  // Whatever failed the leg test may still be a whole pedestrian seen close up.
  for (const auto & c : rest) {
    if (!looksLikePersonBlob(c, p)) {continue;}
    PersonDetection person;
    person.x = c.cx;
    person.y = c.cy;
    person.paired = false;
    // Between a paired detection and a lone distant leg. The evidence is real
    // but weaker: a stance-width round cluster close to the robot.
    person.confidence = 0.6;
    person.separation = c.width;
    people.push_back(person);
  }

  return people;
}

}  // namespace amr_perception
