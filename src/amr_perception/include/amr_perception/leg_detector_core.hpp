// Geometric people detection from a single 2D scan plane.
//
// At the scan height of 150 mm a standing person presents as a pair of small
// round clusters roughly 200 mm apart: the calves. Nothing else in a warehouse
// looks quite like that, which is why leg detection works at all on a plane
// this low.
//
// This is deliberately geometric rather than learned. A detector trained on
// synthetic imagery proves very little about the real world, whereas cluster
// width and separation are physical quantities with the same meaning in both.
// See docs/adr/0002 for the same reasoning applied to the platform itself.
//
// Free of ROS so it unit tests directly.

#ifndef AMR_PERCEPTION__LEG_DETECTOR_CORE_HPP_
#define AMR_PERCEPTION__LEG_DETECTOR_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace amr_perception
{

/// A contiguous run of returns that plausibly belongs to one object.
struct Cluster
{
  std::size_t first{0};
  std::size_t last{0};
  std::size_t points{0};
  double cx{0.0};          ///< centroid x in the scan frame
  double cy{0.0};
  double width{0.0};       ///< straight-line extent from first to last point
  double depth{0.0};       ///< how far the run bows away from that chord
  double range{0.0};       ///< centroid distance from the sensor
};

/// A pair of legs, or a single blob when the legs are together.
struct PersonDetection
{
  double x{0.0};
  double y{0.0};
  double confidence{0.0};
  bool paired{false};      ///< true when two legs were matched
  double separation{0.0};  ///< distance between the two leg centroids
};

struct DetectorParams
{
  /// Base of the adaptive break threshold. Two consecutive returns further
  /// apart than this belong to different objects.
  double cluster_jump_base{0.08};
  /// Growth of that threshold with range. Angular sampling means points spread
  /// out with distance, so a fixed threshold splits far objects into fragments
  /// and merges near ones. This term is that spreading.
  double cluster_jump_slope{3.0};
  std::size_t min_points{3};

  double leg_width_min{0.040};
  double leg_width_max{0.250};
  /// A leg is round, so its run bows away from its own chord. A flat run of the
  /// same width is a wall or a pallet edge.
  double leg_min_depth_ratio{0.08};

  double pair_separation_min{0.05};
  double pair_separation_max{0.55};
  /// Beyond this a single unpaired cluster is not reported at all: at long
  /// range a leg is only a handful of points and the false positive rate from
  /// clutter climbs quickly.
  double single_leg_max_range{4.0};
};

/// Break a scan into clusters using an adaptive distance threshold.
///
/// `ranges` may contain non-finite entries; those break the run, which is
/// correct, since a gap in the data is not evidence of continuity.
std::vector<Cluster> segmentScan(
  const std::vector<float> & ranges,
  double angle_min, double angle_increment,
  const DetectorParams & p);

/// Does this cluster look like a leg rather than a wall, a pallet or noise?
bool looksLikeLeg(const Cluster & c, const DetectorParams & p);

/// Pair up leg-like clusters into people.
std::vector<PersonDetection> pairLegs(
  const std::vector<Cluster> & legs, const DetectorParams & p);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__LEG_DETECTOR_CORE_HPP_
