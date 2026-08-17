// Copyright 2026 Mohamed Kamel
//
// Finding a V shaped dock in a laser scan, and refusing to find one that is
// not there.
//
// WHY A V AND NOT A FLAT PLATE
//
// A flat plate gives range and heading and nothing else. Sliding along a flat
// plate does not change any range reading, so the lateral position of the
// vehicle relative to the plate is UNOBSERVABLE, and a docking controller
// closing on it would drift sideways until it ran out of plate. Two plates at a
// known angle fix all three degrees of freedom: their intersection is a point,
// and the bisector of their directions is a heading.
//
// WHY THE SCAN CAN DO THIS AT ALL
//
// The scanner resolves 0.17 degrees, which is 3.0 mm of beam spacing at one
// metre, so a 0.5 m dock face is covered by about 169 returns. Fitting a line
// through 169 points is far more precise than any single range reading, which
// is the whole reason a docking claim can be made from a sensor that publishes
// a 70 mm detection resolution: that figure is about detecting an object, not
// about locating a surface.
//
// WHY THIS IS THE POINT OF THE EXERCISE
//
// V-62 measured the vehicle parking a median 117 mm from a station, with a
// localisation floor of 55 mm, against a docking requirement of roughly 10 mm.
// A goal in the map frame cannot do better than the map, so the error has to
// stop being a localisation error and become a SENSOR error. That is what this
// converts it into.
//
// WHAT IT MUST REFUSE
//
// Every rack leg, pallet corner and doorway in a warehouse is two surfaces
// meeting at an angle. A detector that reports the first corner it sees will
// dock the vehicle to the building. The validation gates below are therefore
// not defensive programming, they are the feature.

#ifndef AMR_PERCEPTION__DOCK_CORE_HPP_
#define AMR_PERCEPTION__DOCK_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace amr_perception
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

/// A straight line fitted to points, in the form of a unit direction and a
/// point on it. Total least squares, so a near vertical face is no harder than
/// a near horizontal one; fitting y = mx + c would diverge on one of them.
struct Line2
{
  Point2 origin;         ///< the centroid of the points
  double dx{1.0};        ///< unit direction
  double dy{0.0};
  double residual{0.0};  ///< RMS perpendicular distance, metres
  std::size_t n{0};
};

/// The dock's pose in the frame the points were given in.
struct DockPose
{
  double x{0.0};         ///< the apex, where the two faces meet
  double y{0.0};
  double yaw{0.0};       ///< the bisector, pointing OUT of the V toward the vehicle
  double opening{0.0};   ///< measured angle between the faces, radians
  double residual{0.0};  ///< worst of the two line fits
};

/// What a dock is allowed to look like. Everything here is a rejection
/// criterion, and the defaults are for the generated test track's dock.
struct DockSpec
{
  double opening{M_PI / 2.0};     ///< 90 degrees between the two faces
  double opening_tol{0.26};       ///< +/- 15 degrees before it is not a dock
  double min_range{0.25};         ///< inside this the faces leave the aperture
  double max_range{2.50};         ///< beyond this the fit is not worth trusting
  double half_sector{M_PI / 4.0}; ///< only look forward, +/- 45 degrees
  std::size_t min_points{12};     ///< per face
  double max_residual{0.02};      ///< 20 mm RMS; a wall is straighter than this
  double min_face{0.10};          ///< metres of face, or it is a corner not a dock
};

/// Total least squares line fit. Empty result if fewer than two points.
inline std::optional<Line2> fitLine(const std::vector<Point2> & pts)
{
  if (pts.size() < 2) {
    return std::nullopt;
  }
  double mx = 0.0, my = 0.0;
  for (const auto & p : pts) {
    mx += p.x;
    my += p.y;
  }
  mx /= static_cast<double>(pts.size());
  my /= static_cast<double>(pts.size());

  double sxx = 0.0, syy = 0.0, sxy = 0.0;
  for (const auto & p : pts) {
    const double ax = p.x - mx, ay = p.y - my;
    sxx += ax * ax;
    syy += ay * ay;
    sxy += ax * ay;
  }
  // The principal direction is the eigenvector of the scatter matrix with the
  // larger eigenvalue. atan2 of the doubled angle avoids the degenerate case
  // that a slope form has when the points are vertical.
  const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  Line2 line;
  line.origin = {mx, my};
  line.dx = std::cos(theta);
  line.dy = std::sin(theta);
  line.n = pts.size();

  double sum = 0.0;
  for (const auto & p : pts) {
    // Perpendicular distance to the fitted line.
    const double d = -line.dy * (p.x - mx) + line.dx * (p.y - my);
    sum += d * d;
  }
  line.residual = std::sqrt(sum / static_cast<double>(pts.size()));
  return line;
}

/// Length of the point run along its own fitted direction.
inline double extent(const std::vector<Point2> & pts, const Line2 & line)
{
  if (pts.empty()) {
    return 0.0;
  }
  double lo = 1e9, hi = -1e9;
  for (const auto & p : pts) {
    const double s = line.dx * (p.x - line.origin.x) + line.dy * (p.y - line.origin.y);
    lo = std::min(lo, s);
    hi = std::max(hi, s);
  }
  return hi - lo;
}

/// Find the V, or return nothing.
///
/// The split is by BEARING about the FARTHEST return rather than by clustering.
///
/// The dock is a funnel opening toward the vehicle, which is the mechanical
/// arrangement a robot noses into, so range DECREASES from the apex out along
/// each face and the apex is the deepest return on the target. The first
/// version of this used the nearest return, on the unexamined assumption that
/// the closest point of a thing is the middle of it. That splits at a mouth
/// edge and puts both faces on one side of the split, and the three positive
/// detection tests failed while all six rejection tests passed, which is the
/// signature of a detector that has stopped detecting rather than one that is
/// unsafe.
///
/// Clustering would have to decide where one face ends and the other begins,
/// which is the same problem with more parameters.
inline std::optional<DockPose> findDock(
  const std::vector<Point2> & scan, const DockSpec & spec = {})
{
  // Window first. Everything outside it cannot be the dock and only makes the
  // split harder.
  std::vector<Point2> window;
  window.reserve(scan.size());
  for (const auto & p : scan) {
    const double r = std::hypot(p.x, p.y);
    if (r < spec.min_range || r > spec.max_range) {
      continue;
    }
    if (std::abs(std::atan2(p.y, p.x)) > spec.half_sector) {
      continue;
    }
    window.push_back(p);
  }
  if (window.size() < 2 * spec.min_points) {
    return std::nullopt;
  }

  // The FARTHEST return is on the apex, so its bearing splits the faces.
  const auto apex = std::max_element(
    window.begin(), window.end(),
    [](const Point2 & a, const Point2 & b) {
      return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
    });
  const double split = std::atan2(apex->y, apex->x);

  std::vector<Point2> left, right;
  for (const auto & p : window) {
    const double b = std::atan2(p.y, p.x);
    (b >= split ? left : right).push_back(p);
  }
  if (left.size() < spec.min_points || right.size() < spec.min_points) {
    return std::nullopt;
  }

  const auto la = fitLine(left);
  const auto lb = fitLine(right);
  if (!la || !lb) {
    return std::nullopt;
  }
  if (la->residual > spec.max_residual || lb->residual > spec.max_residual) {
    return std::nullopt;      // not two straight faces
  }
  if (extent(left, *la) < spec.min_face || extent(right, *lb) < spec.min_face) {
    return std::nullopt;      // a corner, not a dock
  }

  // Angle between the faces, folded into [0, pi].
  double between = std::abs(std::atan2(
      la->dx * lb->dy - la->dy * lb->dx,
      la->dx * lb->dx + la->dy * lb->dy));
  if (between > M_PI / 2.0) {
    between = M_PI - between;
  }
  const double opening = M_PI - between;
  if (std::abs(opening - spec.opening) > spec.opening_tol) {
    return std::nullopt;      // the wrong shape
  }

  // Intersect the two fitted lines. Parallel faces cannot meet, and the
  // opening gate above has already made that nearly impossible, but a
  // determinant near zero would still produce an apex at infinity.
  const double det = la->dx * (-lb->dy) - la->dy * (-lb->dx);
  if (std::abs(det) < 1e-9) {
    return std::nullopt;
  }
  const double rx = lb->origin.x - la->origin.x;
  const double ry = lb->origin.y - la->origin.y;
  const double t = (rx * (-lb->dy) - ry * (-lb->dx)) / det;

  DockPose pose;
  pose.x = la->origin.x + t * la->dx;
  pose.y = la->origin.y + t * la->dy;
  // The bisector points from the apex back toward the sensor, which is the
  // direction a vehicle approaches along.
  pose.yaw = std::atan2(-pose.y, -pose.x);
  pose.opening = opening;
  pose.residual = std::max(la->residual, lb->residual);
  return pose;
}

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__DOCK_CORE_HPP_
