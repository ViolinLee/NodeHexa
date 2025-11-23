#include "movement_profile.h"

#include <Arduino.h>

namespace motion {

using namespace hexapod;

namespace {

// 这些数值依据 pathTool 的参数（半径、旋转幅度等）换算得到。
// 如有需要可在实际标定后微调，以米和度为单位。
constexpr MovementMetrics kMetrics[MOVEMENT_TOTAL] = {
    /* MOVEMENT_STANDBY     */ {0.0f, 0.0f, 1.0f},
    /* MOVEMENT_FORWARD     */ {0.050f, 0.0f, 2.0f},
    /* MOVEMENT_FORWARDFAST */ {0.100f, 0.0f, 2.0f},
    /* MOVEMENT_BACKWARD    */ {0.050f, 0.0f, 2.0f},
    /* MOVEMENT_TURNLEFT    */ {0.0f, 30.0f, 2.0f},
    /* MOVEMENT_TURNRIGHT   */ {0.0f, 30.0f, 2.0f},
    /* MOVEMENT_SHIFTLEFT   */ {0.050f, 0.0f, 2.0f},
    /* MOVEMENT_SHIFTRIGHT  */ {0.050f, 0.0f, 2.0f},
    /* MOVEMENT_CLIMB       */ {0.040f, 0.0f, 2.0f},
    /* MOVEMENT_ROTATEX     */ {0.0f, 15.0f, 2.0f},
    /* MOVEMENT_ROTATEY     */ {0.0f, 15.0f, 2.0f},
    /* MOVEMENT_ROTATEZ     */ {0.0f, 20.0f, 2.0f},
    /* MOVEMENT_TWIST       */ {0.0f, 15.0f, 2.0f},
};

}

const MovementMetrics& getMovementMetrics(MovementMode mode) {
    auto index = static_cast<size_t>(mode);
    if (index >= MOVEMENT_TOTAL) {
        static MovementMetrics fallback{0.0f, 0.0f, 1.0f};
        return fallback;
    }
    return kMetrics[index];
}

}

