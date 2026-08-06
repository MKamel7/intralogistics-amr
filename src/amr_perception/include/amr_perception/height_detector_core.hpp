// People detection from depth, using the one thing a scan plane cannot see:
// how tall something is and how its width changes with height.
//
// The leg detector reaches precision 0.168 because a rack upright is a round,
// leg-sized cylinder on a plane 150 mm off the floor. Tracking lifts that to
// 0.312 by discarding anything that does not move, which also discards anyone
// standing still. Neither can separate a standing person from a post, because
// on their evidence the two are identical.
//
// Height separates them. A person is narrow at the calves, wide at the torso,
// and stops at about 1.75 m. A rack upright is the same width all the way up
// and keeps going. That profile is what this file measures.
//
// Free of ROS so it unit tests directly. The caller supplies the transform from
// the camera optical frame into the robot frame; everything here is arithmetic.

#ifndef AMR_PERCEPTION__HEIGHT_DETECTOR_CORE_HPP_
#define AMR_PERCEPTION__HEIGHT_DETECTOR_CORE_HPP_

#include <cstddef>
#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace amr_perception
{

/// Pinhole intrinsics, as published on camera_info.
struct Intrinsics
{
  double fx{1.0};
  double fy{1.0};
  double cx{0.0};
  double cy{0.0};
  std::size_t width{0};
  std::size_t height{0};
};

/// A rigid transform, camera optical frame into the robot frame.
struct Transform3D
{
  /// Row-major 3x3 rotation.
  std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  std::array<double, 3> t{{0, 0, 0}};

  inline void apply(double x, double y, double z, double & ox, double & oy, double & oz) const
  {
    ox = R[0] * x + R[1] * y + R[2] * z + t[0];
    oy = R[3] * x + R[4] * y + R[5] * z + t[1];
    oz = R[6] * x + R[7] * y + R[8] * z + t[2];
  }
};

struct HeightParams
{
  /// Every Nth pixel in each direction. A 640 x 480 depth image is 307k points;
  /// at 15 Hz on two cameras that is 9 million points a second, which this CPU
  /// does not have. Stride 4 keeps 19k per image, which is ample for finding
  /// something the size of a person.
  std::size_t stride{4};

  double min_depth{0.105};      ///< sensor Min-Z
  double max_depth{10.0};       ///< sensor maximum range
  double max_range{6.0};        ///< beyond this a person is too few points to trust

  /// Anything below this is floor. Not zero: depth noise puts points slightly
  /// under the ground plane and they would otherwise form a giant cluster.
  double ground_z{0.08};
  /// Anything above this is roof, lighting or racking tops.
  double ceiling_z{2.60};

  /// Camera geometry, needed to know what the camera COULD have seen.
  ///
  /// This matters more than it looks. With a 58 degree vertical field of view
  /// mounted at 0.27 m, the highest visible point is 0.27 + r*tan(29 deg): only
  /// 1.10 m at 1.5 m range and 1.66 m at 2.5 m. Closer than about 2.7 m every
  /// tall object is cut off at the same apparent height, so a 3 m rack upright
  /// looks exactly as tall as a person. Any conclusion drawn from "it tops out
  /// where a person tops out" is worthless there, and the detector has to know
  /// that rather than quietly guess.
  double camera_height{0.27};
  double camera_vfov{58.0};       ///< degrees
  /// How close to the visible ceiling counts as cut off.
  double truncation_margin{0.12};

  double cell_size{0.15};       ///< horizontal grid resolution for clustering
  std::size_t min_points{12};

  /// Bands used for the width profile, in metres above the floor.
  double leg_band_low{0.10};
  double leg_band_high{0.55};
  double torso_band_low{0.95};
  double torso_band_high{1.55};

  /// A person tops out around here; a rack upright does not.
  double person_top_min{1.10};
  double person_top_max{2.10};
  /// A person is wider at the torso than at the calves. A post is not.
  double torso_widening{1.30};
  /// Nothing person shaped is wider than this.
  double max_width{1.00};
};

/// A vertical column of depth returns, grouped horizontally.
struct HeightCluster
{
  double cx{0.0};               ///< centroid in the robot frame
  double cy{0.0};
  double min_z{0.0};
  double max_z{0.0};
  double width{0.0};            ///< horizontal extent
  double leg_width{0.0};        ///< extent of points in the leg band
  double torso_width{0.0};      ///< extent of points in the torso band
  std::size_t points{0};
  std::size_t leg_points{0};
  std::size_t torso_points{0};
  double range{0.0};
  /// True when the cluster reaches the top of what the camera can see at its
  /// range, so its real height is unknown and could be anything larger.
  bool truncated{false};
  double visible_ceiling{0.0};    ///< highest z the camera could see there
};

/// Highest point the camera can see at horizontal distance `range`.
double visibleCeiling(double range, const HeightParams & p);

/// Unproject a depth image, drop floor and ceiling, and group what is left into
/// horizontal clusters with a height profile.
///
/// `depth` is metres, row major, `width * height` entries. Non-finite and
/// non-positive entries are treated as no return.
std::vector<HeightCluster> clusterDepth(
  const std::vector<float> & depth, const Intrinsics & intr,
  const Transform3D & camera_to_robot, const HeightParams & p);

/// Does this column look like a person rather than structure?
bool looksLikePerson(const HeightCluster & c, const HeightParams & p);

/// Does it look like something that continues past a person's height, which is
/// what a rack upright, a wall or a door frame does?
bool looksLikeStructure(const HeightCluster & c, const HeightParams & p);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__HEIGHT_DETECTOR_CORE_HPP_
