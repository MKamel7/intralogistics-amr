// Multi-object tracking for pedestrians, on a constant-velocity model.
//
// WHAT TRACKING BUYS, AND WHAT IT DOES NOT
//
// It is tempting to say that tracking removes the static false positives the
// leg detector produces. It does not, and the reason matters. Track
// confirmation counts how often something is seen, and a rack upright is seen
// in every single frame: it is the MOST confirmable thing in the warehouse.
// M-of-N confirmation would promote it immediately.
//
// What separates structure from a pedestrian here is the VELOCITY ESTIMATE.
// A tracked upright has a speed indistinguishable from zero; a walking person
// does not. So tracks carry a moving/stationary classification and the caller
// decides what to trust.
//
// That leaves an honest hole: a person standing still is also stationary, and
// on a single scan plane is genuinely indistinguishable from a post. Only the
// height channel can separate those. This file does not pretend otherwise.
//
// Free of ROS so it unit tests directly.

#ifndef AMR_PERCEPTION__TRACKER_CORE_HPP_
#define AMR_PERCEPTION__TRACKER_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace amr_perception
{

/// A position measurement in the tracking frame.
struct Observation
{
  double x{0.0};
  double y{0.0};
  double confidence{1.0};
};

struct TrackerParams
{
  /// Process noise: how much the constant-velocity assumption is allowed to be
  /// wrong per second. A walking person changes direction, so this is not small.
  double accel_noise{1.2};        // m/s^2
  /// Measurement noise, from the measured leg-detector localisation error:
  /// p50 was 5.4 cm and p95 33 cm, so one sigma of about 0.12 m is fair.
  double measurement_noise{0.12};  // m

  /// Association gate, as a Mahalanobis distance. Squared-distance gating in
  /// metres would be wrong: a track that has been coasting through an occlusion
  /// is far less certain of itself and must be allowed a wider search.
  double gate_mahalanobis{3.0};

  /// A track must be seen this many times out of its first `confirm_window`
  /// updates before it is reported. Suppresses one-frame clutter.
  std::size_t confirm_hits{3};
  std::size_t confirm_window{5};

  /// Consecutive misses tolerated before a track is dropped. At 14 Hz this is
  /// roughly a second of full occlusion, which is a person passing behind a
  /// rack upright.
  std::size_t max_misses{14};

  /// Speed above which a track is called moving. Below the leg detector's own
  /// localisation noise this cannot be measured, so the threshold sits above it.
  double moving_speed{0.25};       // m/s
  /// ...sustained for this long, so a single noisy update cannot promote a post
  /// into a pedestrian.
  double moving_hold{0.5};         // s
};

/// One tracked object. State is [x, y, vx, vy].
struct Track
{
  std::uint32_t id{0};
  std::array<double, 4> x{{0.0, 0.0, 0.0, 0.0}};
  /// Covariance, row major 4x4.
  std::array<double, 16> P{{0.0}};

  std::size_t hits{0};
  std::size_t misses{0};
  std::size_t updates{0};
  double age{0.0};              ///< seconds since birth
  double moving_for{0.0};       ///< seconds continuously above moving_speed
  bool confirmed{false};

  double speed() const;
  bool isMoving(const TrackerParams & p) const;
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(const TrackerParams & params = TrackerParams())
  : params_(params) {}

  /// Advance every track by `dt` seconds and grow its uncertainty.
  void predict(double dt);

  /// Associate observations to tracks, update, coast the unmatched, spawn new
  /// tracks for unexplained observations, and drop the dead.
  void update(const std::vector<Observation> & observations, double dt);

  const std::vector<Track> & tracks() const {return tracks_;}
  /// Confirmed tracks only, which is what a consumer should act on.
  std::vector<Track> confirmedTracks() const;

  const TrackerParams & params() const {return params_;}
  std::uint32_t nextId() const {return next_id_;}

private:
  void spawn(const Observation & obs);

  TrackerParams params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
};

/// Squared Mahalanobis distance between a track's predicted position and an
/// observation, using the position block of the covariance plus measurement
/// noise. Exposed for testing the gate directly.
double mahalanobisSquared(
  const Track & track, const Observation & obs, double measurement_noise);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__TRACKER_CORE_HPP_
