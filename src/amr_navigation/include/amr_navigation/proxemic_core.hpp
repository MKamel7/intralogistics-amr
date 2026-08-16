// Copyright 2026 Mohamed Kamel
//
// The cost a planner pays for passing close to a person, as pure arithmetic.
//
// WHY THIS IS SEPARATE FROM THE LAYER
//
// A nav2 costmap layer cannot be instantiated without a costmap, a tf buffer
// and a lifecycle node, so anything living inside one is only testable by
// bringing up the stack. The shape of this cost is the part worth arguing
// about and it is checkable in microseconds, so it lives here.
//
// WHY A COST AND NOT AN OBSTACLE
//
// People are already obstacles: the scan sees them and the obstacle layer
// marks them, which is why the vehicle has never driven into anybody (V-51).
// What it does not do is prefer the wider way round. A person passed at 40 mm
// and a person passed at 900 mm cost the planner exactly the same today, and
// the proxemic figures in V-43 exist to make that difference measurable.
//
// THE SHAPE, AND WHY NOT A GAUSSIAN
//
// The obvious choice is a Gaussian, and it is wrong here for a reason worth
// recording: it never reaches zero, so every cell in the costmap carries some
// cost from every person, and the planner's tie breaking degenerates. It also
// has no natural radius, so the parameter that matters becomes a standard
// deviation nobody can tie to a distance.
//
// This is a smooth falloff to EXACTLY zero at a stated radius, so the radius
// is the parameter and it is in metres:
//
//     cost(d) = peak * (1 - (d / radius))^2      for d < radius
//             = 0                                otherwise
//
// The square makes it soft near the edge, where a person merely being nearby
// should barely register, and steep near the centre, where the vehicle should
// be strongly discouraged. It is monotonic, which the planner needs.

#ifndef AMR_NAVIGATION__PROXEMIC_CORE_HPP_
#define AMR_NAVIGATION__PROXEMIC_CORE_HPP_

#include <algorithm>
#include <cmath>

namespace amr_navigation
{

/// Cost contributed by one person at distance `d` metres.
///
/// \param peak cost at the person's own position, 0 to 252. It is capped
///        BELOW the costmap's lethal and inscribed values on purpose: this
///        layer must never make a cell untraversable. A person standing in a
///        doorway would otherwise close the only route and the vehicle would
///        abort the mission rather than wait, which is worse behaviour than
///        passing them politely.
inline double proxemicCost(double d, double radius, double peak)
{
  if (!(radius > 0.0) || d >= radius || d < 0.0) {
    return 0.0;
  }
  const double t = 1.0 - (d / radius);
  return peak * t * t;
}

/// The largest cost this layer may ever write.
///
/// nav2_costmap_2d: LETHAL_OBSTACLE is 254 and INSCRIBED_INFLATED_OBSTACLE is
/// 253. Anything at or above the inscribed value tells the planner the robot's
/// body cannot be there, which is a claim about geometry. This layer makes a
/// claim about preference, so it stops below both.
inline constexpr unsigned char kMaxProxemicCost = 252;

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__PROXEMIC_CORE_HPP_
