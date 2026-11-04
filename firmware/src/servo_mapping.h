#pragma once

#include <algorithm>
#include <cmath>

namespace motioncore {

struct AngleLimits {
    float min;
    float max;
    bool contains(float angle) const {
        return std::isfinite(angle) && angle >= min && angle <= max;
    }
};

// Preserve each board's established pulse gain and integer rounding. Calibration
// is in electrical degrees; adjust/inverse convert geometric joint angles.
struct ServoMapping {
    bool inverse;
    float adjust;
    float range;
    int offset;
    bool quad;

    AngleLimits limits() const {
        const float electricalLimit = quad ? 90.0f : 1000.0f / 11.0f;
        const float sign = inverse ? -1.0f : 1.0f;
        float a = adjust + sign * (-electricalLimit - offset);
        float b = adjust + sign * ( electricalLimit - offset);
        if (a > b) std::swap(a, b);
        return {std::max(adjust - range, a), std::min(adjust + range, b)};
    }

    bool pulse(float angle, int& ticks) const {
        if (!limits().contains(angle)) return false;
        const float electrical = (inverse ? -(angle - adjust) : angle - adjust) + offset;
        const int us = quad
            ? 1500 + static_cast<int>(electrical * (1000 / 90.0f))
            : static_cast<int>(1500 + electrical * (1000 / 90));
        if (us < 500 || us > 2500) return false;
        ticks = us / 5;
        return true;
    }
};

} // namespace motioncore
