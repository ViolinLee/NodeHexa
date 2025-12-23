#pragma once

#include "robot.h"
#include "quad_leg.h"
#include "quad_movement.h"

namespace quadruped {

    // NodeQuadMini 机器人骨架，实现与 Hexapod 相同的 RobotBase 接口
    class QuadRobot : public hexapod::RobotBase {
    public:
        QuadRobot();
        virtual ~QuadRobot() = default;

        // 初始化
        void init(bool setting, bool isReset = false) override;

        // 运动
        void processMovement(hexapod::MovementMode mode, int elapsedMs = 0) override;

        void setMovementSpeed(float speed) override;
        void setMovementSpeedLevel(hexapod::SpeedLevel level) override;
        float getMovementSpeed() const override;

        // 校准
        void calibrationSave() override;
        void calibrationGet(int legIndex, int partIndex, int& offset) override;
        void calibrationSet(int legIndex, int partIndex, int offset) override;
        void calibrationSet(CalibrationData& calibrationData) override;
        void calibrationTest(int legIndex, int partIndex, float angle) override;
        void calibrationTestAllLeg(float angle) override;
        void clearOffset() override;
        void forceResetAllLegTippos() override;

        // 步态模式控制：0-trot,1-walk,2-gallop,3-creep
        void setGaitMode(int gaitMode) override;

    private:
        void calibrationLoad();

    private:
        // 校准文件（四足）
        static constexpr const char* kCalibrationFilePath = "/calibration_quad.json";

        float speed_;

        Leg legs_[4];
        hexapod::MovementMode mode_;
        QuadMovement movement_;
    };

} // namespace quadruped

