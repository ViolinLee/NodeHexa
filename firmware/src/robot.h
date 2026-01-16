#pragma once

#include <cstdint>

#include "movement.h"
#include "calibration.h"
#include "config.h"

namespace hexapod {

    // 统一的机器人控制抽象接口
    class RobotBase {
    public:
        virtual ~RobotBase() = default;

        // 初始化
        virtual void init(bool setting, bool isReset = false) = 0;

        // 运动相关
        virtual void processMovement(MovementMode mode, int elapsedMs = 0) = 0;

        virtual void setMovementSpeed(float speed) = 0;
        virtual void setMovementSpeedLevel(SpeedLevel level) = 0;
        virtual float getMovementSpeed() const = 0;

        // 运动周期时长（用于运动规划：将 elapsedMs 归一化到 cycles）
        // 返回值单位：ms（可能为小数）
        virtual float getMovementCycleDurationMs(MovementMode mode) const = 0;

        // 校准相关
        virtual void calibrationSave() = 0;
        virtual void calibrationGet(int legIndex, int partIndex, int& offset) = 0;
        virtual void calibrationSet(int legIndex, int partIndex, int offset) = 0;
        virtual void calibrationSet(CalibrationData& calibrationData) = 0;
        virtual void calibrationTest(int legIndex, int partIndex, float angle) = 0;
        virtual void calibrationTestAllLeg(float angle) = 0;
        virtual void clearOffset() = 0;
        virtual void forceResetAllLegTippos() = 0;

        // 步态模式控制（默认空实现，保证向后兼容）
        // 对六足模型或不支持多步态的实现，可以忽略此调用。
        virtual void setGaitMode(int gaitMode) {
            (void)gaitMode;
        }
    };

    // 当前正在使用的机器人实例指针
    extern RobotBase* Robot;
}

