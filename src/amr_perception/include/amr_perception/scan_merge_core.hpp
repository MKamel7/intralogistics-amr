// Pure geometry for merging two safety scanners into one 360 degree scan.
//
// Deliberately free of ROS and of TF so it can be unit tested directly. The
// node supplies the transforms; everything here is arithmetic on points.
//
// Why a merge node exists at all: the platform carries two scanners with a 275
// degree aperture each, mounted in diagonally opposite corner recesses. Neither
// sees the whole vehicle surroundings, and the returns arrive in two different
// frames at two different instants. Concatenating the two range arrays would be
// wrong on both counts.

#ifndef AMR_PERCEPTION__SCAN_MERGE_CORE_HPP_
#define AMR_PERCEPTION__SCAN_MERGE_CORE_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace amr_perception
{

/// A 2D rigid transform, enough to move scan points between frames.
struct Transform2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  /// Apply to a point expressed in the source frame.
  inline void apply(double px, double py, double & ox, double & oy) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    ox = c * px - s * py + x;
    oy = s * px + c * py + y;
  }
};

/// Rejects returns that fall inside the vehicle's own envelope.
///
/// Nothing real can be inside the robot, so any such return is the vehicle
/// seeing itself. This is not hypothetical: with the scanners in their corner
/// recesses, the arc pointing inboard reads about 0.06 m, which is the chassis
/// wall. Those returns have to go before the merged scan is usable.
///
/// SHAPED LIKE THE VEHICLE, NOT LIKE ITS BOUNDING BOX, and that distinction is
/// the whole of V-39.
///
/// The scanner pods stand 28.7 mm proud of the chassis at the two diagonal
/// CORNERS. A bounding box that covers them therefore blanks that much
/// everywhere, including along the middle of each side where there is no pod
/// and the vehicle is only its chassis width.
///
/// That mattered because the forward protective fields are only about 65 mm
/// wider than the chassis, being a stopping distance plus the scanner's field
/// supplement. A uniform 32 mm blind margin left them 33 mm of lateral
/// coverage, against the 50 mm a person entering the field needs in order to
/// produce the two returns `min_points` requires. Two attempts to fix that by
/// making the FIELDS bigger both made things worse: V-42 trapped the vehicle
/// against a rack, V-45 dropped the other platform from 3 of 3 cycles to 2 of
/// 9. Making the FILTER smaller where the vehicle is actually smaller costs
/// nothing and closes the gap.
///
/// So: a small margin on the body, and the pods modelled where they are.
class FootprintFilter
{
public:
  struct Pod
  {
    double x{0.0};        ///< centre in base frame
    double y{0.0};
    double half{0.0};     ///< half extent of the housing, already rotated
  };

  /// \param margin clearance around the CHASSIS only. The pods carry their
  ///        own geometry and must not be covered by inflating this.
  FootprintFilter(double length, double width, double margin,
                  std::vector<Pod> pods = {})
  : half_length_(0.5 * length + margin),
    half_width_(0.5 * width + margin),
    pods_(std::move(pods)) {}

  inline bool isSelfReturn(double x, double y) const
  {
    if (std::abs(x) <= half_length_ && std::abs(y) <= half_width_) {
      return true;
    }
    for (const auto & p : pods_) {
      if (std::abs(x - p.x) <= p.half && std::abs(y - p.y) <= p.half) {
        return true;
      }
    }
    return false;
  }

  double halfLength() const {return half_length_;}
  double halfWidth() const {return half_width_;}
  const std::vector<Pod> & pods() const {return pods_;}

private:
  double half_length_;
  double half_width_;
  std::vector<Pod> pods_;
};

/// One scanner's worth of polar data, in that scanner's own frame.
struct ScanView
{
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  const std::vector<float> * ranges{nullptr};
};

/// One merged return, in the output frame.
struct MergedPoint
{
  double x{0.0};
  double y{0.0};
  double bearing{0.0};
};

/// Accumulates points from any number of scanners into a single 360 degree scan.
///
/// Bins take the MINIMUM range, never an average. A safety-relevant scan must
/// report the nearest thing in a direction; averaging a 0.4 m obstacle with a
/// 10 m wall would invent 5 m of clear space that is not there.
///
/// The binned scan is NOT the whole output. Re-binning two scanners mounted off
/// the robot centre about that centre is lossy: the angular mapping is
/// non-uniform, output bins get skipped, and close objects arrive perforated.
/// Measured, a pedestrian at 1.28 m fragmented into runs of one to four points.
/// So every accepted return is also kept as a POINT, un-binned, and anything
/// that needs to cluster should use those. The binned scan remains because a
/// costmap and a collision monitor both want a LaserScan, and for THAT purpose
/// the holes are harmless: a protective field asks whether a return is inside
/// it, not whether its neighbours are contiguous.
class ScanAccumulator
{
public:
  ScanAccumulator(std::size_t bins, double range_min, double range_max)
  : range_min_(range_min), range_max_(range_max),
    ranges_(bins, std::numeric_limits<float>::infinity()) {}

  std::size_t bins() const {return ranges_.size();}
  const std::vector<float> & ranges() const {return ranges_;}
  /// Every accepted return, un-binned and in the output frame.
  const std::vector<MergedPoint> & points() const {return points_;}

  double angleIncrement() const {return 2.0 * M_PI / static_cast<double>(ranges_.size());}
  double angleMin() const {return -M_PI;}

  void reset()
  {
    points_.clear();
    std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());
    accepted_ = 0;
    self_returns_ = 0;
    out_of_range_ = 0;
    invalid_ = 0;
  }

  /// Fold one scanner's returns in, transformed by `tf` into the output frame.
  void add(const ScanView & scan, const Transform2D & tf, const FootprintFilter & footprint)
  {
    if (scan.ranges == nullptr) {return;}
    const std::size_t n = scan.ranges->size();
    for (std::size_t i = 0; i < n; ++i) {
      const double r = static_cast<double>((*scan.ranges)[i]);
      if (!std::isfinite(r)) {++invalid_; continue;}
      if (r < scan.range_min || r > scan.range_max) {++out_of_range_; continue;}

      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      double x, y;
      tf.apply(r * std::cos(a), r * std::sin(a), x, y);

      if (footprint.isSelfReturn(x, y)) {++self_returns_; continue;}

      const double out_r = std::hypot(x, y);
      if (out_r < range_min_ || out_r > range_max_) {++out_of_range_; continue;}

      const double out_a = std::atan2(y, x);
      auto bin = static_cast<long>(std::floor((out_a + M_PI) / angleIncrement()));
      if (bin < 0) {bin = 0;}
      if (bin >= static_cast<long>(ranges_.size())) {bin = static_cast<long>(ranges_.size()) - 1;}

      auto & slot = ranges_[static_cast<std::size_t>(bin)];
      if (static_cast<float>(out_r) < slot) {slot = static_cast<float>(out_r);}
      points_.push_back(MergedPoint{x, y, out_a});
      ++accepted_;
    }
  }

  std::size_t accepted() const {return accepted_;}
  std::size_t selfReturns() const {return self_returns_;}
  std::size_t outOfRange() const {return out_of_range_;}
  std::size_t invalid() const {return invalid_;}

  /// Bins that received no return at all. A large value here is the signal that
  /// the pair of scanners is not actually covering the full turn.
  std::size_t emptyBins() const
  {
    std::size_t n = 0;
    for (const auto r : ranges_) {
      if (!std::isfinite(r)) {++n;}
    }
    return n;
  }

private:
  double range_min_;
  double range_max_;
  std::vector<float> ranges_;
  std::vector<MergedPoint> points_;
  std::size_t accepted_{0};
  std::size_t self_returns_{0};
  std::size_t out_of_range_{0};
  std::size_t invalid_{0};
};

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__SCAN_MERGE_CORE_HPP_
