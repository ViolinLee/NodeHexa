#pragma once

#include <functional>
#include "leg_kinematics.h"

namespace motioncore {
using TipInterpolator = std::function<hexapod::Point3D(int, const hexapod::Point3D&,
                                                    const hexapod::Point3D&, float)>;

// Keep established Cartesian trajectories wherever they are valid. A straight
// segment can cross a hole in the reachable workspace even with valid ends;
// there, interpolate the two calibrated joint solutions instead.
template<class Leg>
hexapod::Point3D interpolateTip(Leg& leg, const hexapod::Point3D& from,
                              const hexapod::Point3D& to, float fraction) {
    auto candidate = from;
    candidate += (to - from) * fraction;
    if (leg.canMoveTip(candidate)) return candidate;
    hexapod::Point3D a, b;
    leg.translateToLocal(from, a);
    leg.translateToLocal(to, b);
    float qa[3], qb[3];
    if (!leg.solveLocal(a, qa) || !leg.solveLocal(b, qb)) return candidate;
    for (int j = 0; j < 3; ++j) qa[j] += (qb[j] - qa[j]) * fraction;
    forward(qa, a);
    leg.translateToWorld(a, candidate);
    return candidate;
}
} // namespace motioncore
