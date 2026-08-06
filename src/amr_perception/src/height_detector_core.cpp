#include "amr_perception/height_detector_core.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace amr_perception
{

namespace
{

/// Accumulated statistics for one horizontal grid cell.
struct Cell
{
  double sx{0.0}, sy{0.0};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
  double leg_min_x{0.0}, leg_max_x{0.0}, leg_min_y{0.0}, leg_max_y{0.0};
  double torso_min_x{0.0}, torso_max_x{0.0}, torso_min_y{0.0}, torso_max_y{0.0};
  double min_x{0.0}, max_x{0.0}, min_y{0.0}, max_y{0.0};
  std::size_t n{0}, leg_n{0}, torso_n{0};

  void add(double x, double y, double z, const HeightParams & p)
  {
    if (n == 0) {
      min_x = max_x = x;
      min_y = max_y = y;
    } else {
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    }
    sx += x; sy += y;
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
    ++n;

    if (z >= p.leg_band_low && z <= p.leg_band_high) {
      if (leg_n == 0) {
        leg_min_x = leg_max_x = x; leg_min_y = leg_max_y = y;
      } else {
        leg_min_x = std::min(leg_min_x, x); leg_max_x = std::max(leg_max_x, x);
        leg_min_y = std::min(leg_min_y, y); leg_max_y = std::max(leg_max_y, y);
      }
      ++leg_n;
    }
    if (z >= p.torso_band_low && z <= p.torso_band_high) {
      if (torso_n == 0) {
        torso_min_x = torso_max_x = x; torso_min_y = torso_max_y = y;
      } else {
        torso_min_x = std::min(torso_min_x, x); torso_max_x = std::max(torso_max_x, x);
        torso_min_y = std::min(torso_min_y, y); torso_max_y = std::max(torso_max_y, y);
      }
      ++torso_n;
    }
  }

  void merge(const Cell & o)
  {
    if (o.n == 0) {return;}
    if (n == 0) {
      *this = o;
      return;
    }
    sx += o.sx; sy += o.sy;
    min_z = std::min(min_z, o.min_z);
    max_z = std::max(max_z, o.max_z);
    min_x = std::min(min_x, o.min_x); max_x = std::max(max_x, o.max_x);
    min_y = std::min(min_y, o.min_y); max_y = std::max(max_y, o.max_y);
    if (o.leg_n) {
      if (leg_n == 0) {
        leg_min_x = o.leg_min_x; leg_max_x = o.leg_max_x;
        leg_min_y = o.leg_min_y; leg_max_y = o.leg_max_y;
      } else {
        leg_min_x = std::min(leg_min_x, o.leg_min_x);
        leg_max_x = std::max(leg_max_x, o.leg_max_x);
        leg_min_y = std::min(leg_min_y, o.leg_min_y);
        leg_max_y = std::max(leg_max_y, o.leg_max_y);
      }
      leg_n += o.leg_n;
    }
    if (o.torso_n) {
      if (torso_n == 0) {
        torso_min_x = o.torso_min_x; torso_max_x = o.torso_max_x;
        torso_min_y = o.torso_min_y; torso_max_y = o.torso_max_y;
      } else {
        torso_min_x = std::min(torso_min_x, o.torso_min_x);
        torso_max_x = std::max(torso_max_x, o.torso_max_x);
        torso_min_y = std::min(torso_min_y, o.torso_min_y);
        torso_max_y = std::max(torso_max_y, o.torso_max_y);
      }
      torso_n += o.torso_n;
    }
    n += o.n;
  }
};

using Key = std::pair<int, int>;

}  // namespace

std::vector<HeightCluster> clusterDepth(
  const std::vector<float> & depth, const Intrinsics & intr,
  const Transform3D & camera_to_robot, const HeightParams & p)
{
  std::map<Key, Cell> grid;
  const std::size_t stride = std::max<std::size_t>(1, p.stride);

  for (std::size_t v = 0; v < intr.height; v += stride) {
    for (std::size_t u = 0; u < intr.width; u += stride) {
      const std::size_t idx = v * intr.width + u;
      if (idx >= depth.size()) {continue;}
      const double d = static_cast<double>(depth[idx]);
      if (!std::isfinite(d) || d <= 0.0) {continue;}
      if (d < p.min_depth || d > p.max_depth) {continue;}

      // Pinhole unprojection in the optical convention: z forward, x right,
      // y down. Mixing this up with the robot convention is the classic way to
      // get a point cloud that looks plausible and is rotated ninety degrees.
      const double xc = (static_cast<double>(u) - intr.cx) * d / intr.fx;
      const double yc = (static_cast<double>(v) - intr.cy) * d / intr.fy;
      const double zc = d;

      double x, y, z;
      camera_to_robot.apply(xc, yc, zc, x, y, z);

      if (z < p.ground_z || z > p.ceiling_z) {continue;}
      const double range = std::hypot(x, y);
      if (range > p.max_range) {continue;}

      const Key key{
        static_cast<int>(std::floor(x / p.cell_size)),
        static_cast<int>(std::floor(y / p.cell_size))};
      grid[key].add(x, y, z, p);
    }
  }

  // Connected components over the eight-neighbourhood of occupied cells.
  std::map<Key, bool> visited;
  std::vector<HeightCluster> clusters;

  for (const auto & entry : grid) {
    if (visited[entry.first]) {continue;}

    Cell merged;
    std::vector<Key> stack{entry.first};
    visited[entry.first] = true;

    while (!stack.empty()) {
      const Key k = stack.back();
      stack.pop_back();
      auto it = grid.find(k);
      if (it == grid.end()) {continue;}
      merged.merge(it->second);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {continue;}
          const Key nb{k.first + dx, k.second + dy};
          if (grid.count(nb) && !visited[nb]) {
            visited[nb] = true;
            stack.push_back(nb);
          }
        }
      }
    }

    if (merged.n < p.min_points) {continue;}

    HeightCluster c;
    c.points = merged.n;
    c.cx = merged.sx / static_cast<double>(merged.n);
    c.cy = merged.sy / static_cast<double>(merged.n);
    c.min_z = merged.min_z;
    c.max_z = merged.max_z;
    c.width = std::max(merged.max_x - merged.min_x, merged.max_y - merged.min_y);
    c.leg_points = merged.leg_n;
    c.torso_points = merged.torso_n;
    c.leg_width = merged.leg_n
      ? std::max(merged.leg_max_x - merged.leg_min_x, merged.leg_max_y - merged.leg_min_y)
      : 0.0;
    c.torso_width = merged.torso_n
      ? std::max(merged.torso_max_x - merged.torso_min_x, merged.torso_max_y - merged.torso_min_y)
      : 0.0;
    c.range = std::hypot(c.cx, c.cy);
    c.visible_ceiling = visibleCeiling(c.range, p);
    c.truncated = c.max_z >= c.visible_ceiling - p.truncation_margin;
    clusters.push_back(c);
  }

  return clusters;
}

double visibleCeiling(double range, const HeightParams & p)
{
  return p.camera_height + range * std::tan(p.camera_vfov * M_PI / 180.0 / 2.0);
}

bool looksLikePerson(const HeightCluster & c, const HeightParams & p)
{
  if (c.points < p.min_points) {return false;}
  // Tops out where a person tops out. A rack upright runs past this, BUT only
  // if the camera could see far enough up to notice. When the cluster is
  // truncated by the vertical field of view its top carries no information and
  // the test is skipped rather than trusted.
  if (!c.truncated && (c.max_z < p.person_top_min || c.max_z > p.person_top_max)) {
    return false;
  }
  if (c.truncated && c.max_z < p.person_top_min) {return false;}
  if (c.width > p.max_width) {return false;}
  // There has to be a torso. This is the part a scan plane cannot see at all.
  if (c.torso_points == 0) {return false;}
  // And it has to be wider than the legs beneath it. A post of uniform width
  // fails here even when its top happens to fall in the right band.
  if (c.leg_points > 0 && c.leg_width > 1e-6) {
    if (c.torso_width < p.torso_widening * c.leg_width) {return false;}
  }
  return true;
}

bool looksLikeStructure(const HeightCluster & c, const HeightParams & p)
{
  // Only assertable when the top was actually observed. A truncated cluster
  // might be a person or a pillar and saying either would be a guess.
  return !c.truncated && c.max_z > p.person_top_max;
}

}  // namespace amr_perception
