#pragma once

#include <algorithm>
#include <cmath>
#include "base.h"
#include "config.h"

namespace motioncore {

inline void forward(const float angles[3], hexapod::Point3D& out) {
    using namespace hexapod::config;
    const float pi = std::acos(-1.0f);
    float r[3];
    for (int i = 0; i < 3; ++i) r[i] = pi * angles[i] / 180;
    const float reach = kLegJoint1ToJoint2 + std::cos(r[1]) * kLegJoint2ToJoint3
        + std::cos(r[1] + r[2] - pi/2) * kLegJoint3ToTip;
    out = {kLegRootToJoint1 + std::cos(r[0]) * reach, std::sin(r[0]) * reach,
           std::sin(r[1]) * kLegJoint2ToJoint3 + std::sin(r[1] + r[2] - pi/2) * kLegJoint3ToTip};
}

inline bool inverse(const hexapod::Point3D& tip, float angles[3]) {
    using namespace hexapod::config;
    if (!std::isfinite(tip.x_) || !std::isfinite(tip.y_) || !std::isfinite(tip.z_)) return false;
    const float pi = std::acos(-1.0f);
    float x = tip.x_ - kLegRootToJoint1, y = tip.y_;
    angles[0] = std::atan2(y, x) * 180 / pi;
    x = std::sqrt(x*x + y*y) - kLegJoint1ToJoint2;
    y = tip.z_;
    const float lr2 = x*x + y*y;
    const float lr = std::sqrt(lr2);
    if (!std::isfinite(lr) || lr < 1e-4f) return false;
    const float ratio1 = (lr2 + kLegJoint2ToJoint3*kLegJoint2ToJoint3 - kLegJoint3ToTip*kLegJoint3ToTip)/(2*kLegJoint2ToJoint3*lr);
    const float ratio2 = (lr2 - kLegJoint2ToJoint3*kLegJoint2ToJoint3 + kLegJoint3ToTip*kLegJoint3ToTip)/(2*kLegJoint3ToTip*lr);
    // Clamp only floating-point roundoff, never project an unreachable target.
    constexpr float epsilon = 1e-6f;
    if (std::fabs(ratio1) > 1 + epsilon || std::fabs(ratio2) > 1 + epsilon) return false;
    const float a1 = std::acos(std::max(-1.0f, std::min(1.0f, ratio1)));
    const float a2 = std::acos(std::max(-1.0f, std::min(1.0f, ratio2)));
    angles[1] = (std::atan2(y, x) + a1) * 180 / pi;
    angles[2] = 90 - (a1 + a2) * 180 / pi;
    return std::isfinite(angles[1]) && std::isfinite(angles[2]);
}

} // namespace motioncore
