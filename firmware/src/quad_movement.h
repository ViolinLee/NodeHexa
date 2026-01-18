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

    enum QuadPendingSwitch {
        QUAD_SWITCH_NONE = 0,
        // 组内切换（forward/backward、turnleft/turnright、shiftleft/shiftright）：等到 entry 再跳表
        QUAD_SWITCH_WAIT_ENTRY_PAIR,
        // 跨组切换或切到 standby：等到 entry -> entry-ground -> 逐腿对齐到目标 entry
        QUAD_SWITCH_WAIT_ENTRY_ALIGN,
    };

    // 四足版本的 Movement，实现与 hexapod::Movement 类似的插值逻辑
    class QuadMovement {
    public:
        explicit QuadMovement(hexapod::MovementMode mode,
                              QuadGaitMode gait = QUAD_GAIT_CREEP);

        void setMode(hexapod::MovementMode newMode);
        void setGaitMode(QuadGaitMode gait);

        const QuadLocations& next(int elapsedMs);

        // 速度控制
        void setSpeed(float speed);
        float getSpeed() const;

        // 返回指定 mode 的单周期时长（用于运动规划单位换算）
        float cycleDurationMsForMode(hexapod::MovementMode mode) const;

    private:
        const QuadMovementTable& currentTable() const;
        const QuadMovementTable& tableForMode(hexapod::MovementMode mode) const;

    private:
        hexapod::MovementMode mode_;
        hexapod::MovementMode requestedMode_;
        QuadPendingSwitch pendingSwitch_{QUAD_SWITCH_NONE};
        QuadGaitMode gaitMode_;

        QuadLocations position_;
        int index_{0};
        int remainTime_{0};
        float speed_;

        // ---- entry-ground：把当前 entry 的悬空腿落地，形成稳定的“四足触地”姿态 ----
        bool grounding_{false};
        QuadLocations groundStart_{};
        QuadLocations groundTarget_{};
        int groundRemainTime_{0};
        int groundTotalTime_{0};

        // ---- 逐腿对齐：从 entry-ground 对齐到目标模式 entry ----
        bool aligning_{false};
        hexapod::MovementMode alignToMode_{hexapod::MOVEMENT_STANDBY};
        int alignToIndex_{0};
        QuadLocations alignTarget_{};
        QuadLocations alignPhaseStart_{};
        QuadLocations alignPhaseTarget_{};
        int alignPhaseTotalTime_{0};
        int alignLegs_[4] {0, 2, 3, 1}; // 默认斜对角顺序（更稳），运行时会动态重排
        int alignLegCount_{0};
        int alignLegPos_{0};
        int alignPhase_{0}; // 0=lift(Z-first), 1=moveXY, 2=lower(to target if needed)
        float alignLiftZ_{0.0f};
    };

} // namespace quadruped

