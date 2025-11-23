#pragma once

#include "movement.h"

namespace motion {

struct MovementMetrics {
    float distancePerCycleMeters;   // 直线/平移位移，单位：米
    float degreesPerCycle;          // 旋转角度（绕控制轴），单位：度
    float stepsPerCycle;            // 每个动作周期包含的步数
};

const MovementMetrics& getMovementMetrics(hexapod::MovementMode mode);

}

