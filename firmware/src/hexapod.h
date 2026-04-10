#pragma once

#include <cstdint>

#include "movement.h"
#include "leg.h"
#include "calibration.h"
#include "config.h"
#include "robot.h"

namespace hexapod {

    class HexapodClass : public RobotBase {
    public:
        HexapodClass();

        // init

        void init(bool setting, bool isReset=false);

        // Movement API

        void processMovement(MovementMode mode, int elapsed = 0);

        // Speed control API
        void setMovementSpeed(float speed);
        void setMovementSpeedLevel(SpeedLevel level);
        float getMovementSpeed() const;
        float getMovementCycleDurationMs(MovementMode mode) const override;

        // Calibration API

        void calibrationSave(); // write to flash
        void calibrationGet(int legIndex, int partIndex, int& offset);  // read servo setting
        void calibrationSet(int legIndex, int partIndex, int offset);    // update servo setting
        void calibrationSet(CalibrationData&  calibrationData);
        void calibrationTest(int legIndex, int partIndex, float angle);             // test servo setting
        void calibrationTestAllLeg(float angle);
        void clearOffset();
        void forceResetAllLegTippos();

        // 单腿控制：六足支持，四足默认不支持
        bool supportsSingleLegControl() const override;
        bool beginSingleLegControl(int legIndex, Point3D& standbyTip) override;
        bool applySingleLegControl(int legIndex, const Point3D& targetTip) override;
        bool singleLegWorldToLocal(int legIndex, const Point3D& worldTip, Point3D& localTip) override;
        bool singleLegLocalToWorld(int legIndex, const Point3D& localTip, Point3D& worldTip) override;
        void endSingleLegControl() override;

    private:
        void calibrationLoad(); // read from flash

    private:
        const char* calibrationFilePath = "/calibration.json";
        MovementMode mode_;
        Movement movement_;
        Leg legs_[6];
    };

    extern HexapodClass Hexapod;
}