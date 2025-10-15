#pragma once

#include "movement.h"
#include "leg.h"
#include "calibration.h"
#include "config.h"

#ifdef USE_REALTIME_GAIT
#include "gait_parameters.h"
#include "realtime_gait.h"
#include "pose_controller.h"
#endif

namespace hexapod {

    class HexapodClass {
    public:
        HexapodClass();

        // init

        void init(bool setting, bool isReset=false);

        // Movement API

#ifdef USE_PREDEFINED_GAIT
        void processMovement(MovementMode mode, int elapsed = 0);

        // Speed control API
        void setMovementSpeed(float speed);
        void setMovementSpeedLevel(SpeedLevel level);
        float getMovementSpeed() const;
#endif

#ifdef USE_REALTIME_GAIT
        // Realtime Gait API
        void setControlMode(ControlMode mode);
        void setGaitParameters(const GaitParameters& params);
        void setVelocity(const Velocity& vel);
        void setBodyPose(const BodyPose& pose);
        void setBodyPitch(float pitch);  // 独立设置pitch（行走模式用）
        void executeTrick(TrickAction action);
        void updateRealtimeGait(int elapsed);
        
        ControlMode getControlMode() const { return controlMode_; }
        const GaitParameters& getGaitParameters() const;
        const Velocity& getVelocity() const;
        const BodyPose& getBodyPose() const;
#endif

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
        Leg legs_[6];
        
#ifdef USE_PREDEFINED_GAIT
        MovementMode mode_;
        Movement movement_;
#endif

#ifdef USE_REALTIME_GAIT
        ControlMode controlMode_;
        RealtimeGait realtimeGait_;
        PoseController poseController_;
        BodyPose currentPose_;
        float walkModePitch_;  // 行走模式下的pitch
#endif
    };

    extern HexapodClass Hexapod;
}