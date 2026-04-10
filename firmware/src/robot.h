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

        // 返回“当前实际执行的运动模式”（用于动作序列的精确计时）。
        // 默认实现：认为实际执行 mode 与请求 mode 一致。
        // 对存在切换过渡/对齐过程的机型（例如四足），应覆盖此函数返回真实执行的 mode。
        virtual MovementMode executedMovementMode(MovementMode requestedMode) const {
            return requestedMode;
        }

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

        // 单腿控制能力（默认不支持，具体机型按需覆盖）
        virtual bool supportsSingleLegControl() const {
            return false;
        }

        virtual bool beginSingleLegControl(int legIndex, Point3D& standbyTip) {
            (void)legIndex;
            (void)standbyTip;
            return false;
        }

        virtual bool applySingleLegControl(int legIndex, const Point3D& targetTip) {
            (void)legIndex;
            (void)targetTip;
            return false;
        }

        virtual bool singleLegWorldToLocal(int legIndex, const Point3D& worldTip, Point3D& localTip) {
            (void)legIndex;
            (void)worldTip;
            (void)localTip;
            return false;
        }

        virtual bool singleLegLocalToWorld(int legIndex, const Point3D& localTip, Point3D& worldTip) {
            (void)legIndex;
            (void)localTip;
            (void)worldTip;
            return false;
        }

        virtual void endSingleLegControl() {}
    };

    // 当前正在使用的机器人实例指针
    extern RobotBase* Robot;
}

