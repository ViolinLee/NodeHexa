#pragma once

#include "movement.h"
#include "quad_tables.h"

namespace quadruped {

    enum QuadGaitMode {
        QUAD_GAIT_TROT = 0,
        QUAD_GAIT_WALK,
        QUAD_GAIT_GALLOP,
        QUAD_GAIT_CREEP,
        QUAD_GAIT_TOTAL
    };

    // 四足版本的 Movement，实现与 hexapod::Movement 类似的插值逻辑
    class QuadMovement {
    public:
        explicit QuadMovement(hexapod::MovementMode mode,
                              QuadGaitMode gait = QUAD_GAIT_TROT);

        void setMode(hexapod::MovementMode newMode);
        void setGaitMode(QuadGaitMode gait);

        const QuadLocations& next(int elapsedMs);

        // 速度控制
        void setSpeed(float speed);
        float getSpeed() const;

    private:
        const QuadMovementTable& currentTable() const;

    private:
        hexapod::MovementMode mode_;
        QuadGaitMode gaitMode_;

        QuadLocations position_;
        int index_{0};
        int remainTime_{0};
        float speed_;
    };

} // namespace quadruped

