#pragma once

#include "movement.h"
#include "leg.h"
#include "calibration.h"
#include "config.h"

namespace hexapod {

    class HexapodClass {
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

        // Calibration API

        void calibrationSave(); // write to flash
        void calibrationGet(int legIndex, int partIndex, int& offset);  // read servo setting
        void calibrationSet(int legIndex, int partIndex, int offset);    // update servo setting
        void calibrationSet(CalibrationData&  calibrationData);
        void calibrationTest(int legIndex, int partIndex, float angle);             // test servo setting
        void calibrationTestAllLeg(float angle);
        void clearOffset();
        void forceResetAllLegTippos();

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