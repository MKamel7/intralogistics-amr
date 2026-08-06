#include "amr_perception/tracker_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_perception
{

namespace
{
inline double & at(std::array<double, 16> & P, int r, int c) {return P[r * 4 + c];}
inline double at(const std::array<double, 16> & P, int r, int c) {return P[r * 4 + c];}
}  // namespace

double Track::speed() const
{
  return std::hypot(x[2], x[3]);
}

bool Track::isMoving(const TrackerParams & p) const
{
  return moving_for >= p.moving_hold && speed() >= p.moving_speed;
}

double mahalanobisSquared(
  const Track & track, const Observation & obs, double measurement_noise)
{
  // Innovation covariance for a position-only measurement: the position block
  // of P plus the measurement noise.
  const double r = measurement_noise * measurement_noise;
  const double s00 = at(track.P, 0, 0) + r;
  const double s01 = at(track.P, 0, 1);
  const double s10 = at(track.P, 1, 0);
  const double s11 = at(track.P, 1, 1) + r;

  const double det = s00 * s11 - s01 * s10;
  if (std::abs(det) < 1e-12) {return std::numeric_limits<double>::infinity();}

  const double dx = obs.x - track.x[0];
  const double dy = obs.y - track.x[1];
  // Inverse of a 2x2.
  const double i00 = s11 / det;
  const double i01 = -s01 / det;
  const double i10 = -s10 / det;
  const double i11 = s00 / det;

  return dx * (i00 * dx + i01 * dy) + dy * (i10 * dx + i11 * dy);
}

void MultiObjectTracker::predict(double dt)
{
  if (dt <= 0.0) {return;}
  const double q = params_.accel_noise * params_.accel_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  for (auto & t : tracks_) {
    // Constant velocity: x += v*dt.
    t.x[0] += t.x[2] * dt;
    t.x[1] += t.x[3] * dt;

    // P = F P F' + Q, written out for the 4x4 constant-velocity F rather than
    // pulling in a matrix library for sixteen numbers.
    std::array<double, 16> FP{};
    for (int c = 0; c < 4; ++c) {
      at(FP, 0, c) = at(t.P, 0, c) + dt * at(t.P, 2, c);
      at(FP, 1, c) = at(t.P, 1, c) + dt * at(t.P, 3, c);
      at(FP, 2, c) = at(t.P, 2, c);
      at(FP, 3, c) = at(t.P, 3, c);
    }
    std::array<double, 16> P{};
    for (int r = 0; r < 4; ++r) {
      at(P, r, 0) = at(FP, r, 0) + dt * at(FP, r, 2);
      at(P, r, 1) = at(FP, r, 1) + dt * at(FP, r, 3);
      at(P, r, 2) = at(FP, r, 2);
      at(P, r, 3) = at(FP, r, 3);
    }
    // Piecewise white acceleration noise.
    at(P, 0, 0) += q * dt4 / 4.0;
    at(P, 1, 1) += q * dt4 / 4.0;
    at(P, 0, 2) += q * dt3 / 2.0;
    at(P, 2, 0) += q * dt3 / 2.0;
    at(P, 1, 3) += q * dt3 / 2.0;
    at(P, 3, 1) += q * dt3 / 2.0;
    at(P, 2, 2) += q * dt2;
    at(P, 3, 3) += q * dt2;

    t.P = P;
    t.age += dt;
  }
}

void MultiObjectTracker::spawn(const Observation & obs)
{
  Track t;
  t.id = next_id_++;
  t.x = {obs.x, obs.y, 0.0, 0.0};
  const double r = params_.measurement_noise * params_.measurement_noise;
  // Position is known to measurement accuracy; velocity is completely unknown,
  // so it starts wide rather than at zero confidence in zero speed.
  at(t.P, 0, 0) = r;
  at(t.P, 1, 1) = r;
  at(t.P, 2, 2) = 4.0;
  at(t.P, 3, 3) = 4.0;
  t.hits = 1;
  t.updates = 1;
  tracks_.push_back(t);
}

void MultiObjectTracker::update(const std::vector<Observation> & observations, double dt)
{
  predict(dt);

  const double gate2 = params_.gate_mahalanobis * params_.gate_mahalanobis;
  std::vector<bool> obs_used(observations.size(), false);
  std::vector<bool> track_updated(tracks_.size(), false);

  // Global nearest neighbour: repeatedly take the closest admissible pair.
  // With pedestrian densities in a warehouse this is equivalent to a full
  // assignment and far simpler to reason about.
  while (true) {
    double best = gate2;
    std::size_t best_t = tracks_.size();
    std::size_t best_o = observations.size();

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (track_updated[ti]) {continue;}
      for (std::size_t oi = 0; oi < observations.size(); ++oi) {
        if (obs_used[oi]) {continue;}
        const double d2 = mahalanobisSquared(
          tracks_[ti], observations[oi], params_.measurement_noise);
        if (d2 < best) {
          best = d2;
          best_t = ti;
          best_o = oi;
        }
      }
    }
    if (best_t == tracks_.size()) {break;}

    // Kalman update with a position-only measurement.
    Track & t = tracks_[best_t];
    const Observation & z = observations[best_o];
    const double r = params_.measurement_noise * params_.measurement_noise;

    const double s00 = at(t.P, 0, 0) + r;
    const double s01 = at(t.P, 0, 1);
    const double s10 = at(t.P, 1, 0);
    const double s11 = at(t.P, 1, 1) + r;
    const double det = s00 * s11 - s01 * s10;
    if (std::abs(det) > 1e-12) {
      const double i00 = s11 / det, i01 = -s01 / det;
      const double i10 = -s10 / det, i11 = s00 / det;

      // K = P H' S^-1, with H selecting position.
      std::array<double, 8> K{};   // 4x2
      for (int row = 0; row < 4; ++row) {
        const double p0 = at(t.P, row, 0);
        const double p1 = at(t.P, row, 1);
        K[row * 2 + 0] = p0 * i00 + p1 * i10;
        K[row * 2 + 1] = p0 * i01 + p1 * i11;
      }

      const double yx = z.x - t.x[0];
      const double yy = z.y - t.x[1];
      const double prev_speed = t.speed();
      for (int row = 0; row < 4; ++row) {
        t.x[static_cast<std::size_t>(row)] += K[row * 2 + 0] * yx + K[row * 2 + 1] * yy;
      }
      (void)prev_speed;

      // P = (I - K H) P
      std::array<double, 16> P{};
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          at(P, row, col) = at(t.P, row, col)
            - K[row * 2 + 0] * at(t.P, 0, col)
            - K[row * 2 + 1] * at(t.P, 1, col);
        }
      }
      t.P = P;
    }

    ++t.hits;
    ++t.updates;
    t.misses = 0;
    track_updated[best_t] = true;
    obs_used[best_o] = true;
  }

  // Coast whatever was not matched.
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (track_updated[ti]) {continue;}
    ++tracks_[ti].misses;
    ++tracks_[ti].updates;
  }

  // Motion classification, and confirmation.
  for (auto & t : tracks_) {
    if (t.speed() >= params_.moving_speed) {
      t.moving_for += dt;
    } else {
      t.moving_for = 0.0;
    }
    if (!t.confirmed && t.updates <= params_.confirm_window &&
      t.hits >= params_.confirm_hits)
    {
      t.confirmed = true;
    }
  }

  // Drop the dead, and drop anything that failed to confirm inside its window.
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & t) {
        if (t.misses > params_.max_misses) {return true;}
        return !t.confirmed && t.updates > params_.confirm_window;
      }),
    tracks_.end());

  // Anything left over starts a new track.
  for (std::size_t oi = 0; oi < observations.size(); ++oi) {
    if (!obs_used[oi]) {spawn(observations[oi]);}
  }
}

std::vector<Track> MultiObjectTracker::confirmedTracks() const
{
  std::vector<Track> out;
  for (const auto & t : tracks_) {
    if (t.confirmed) {out.push_back(t);}
  }
  return out;
}

}  // namespace amr_perception
