#pragma once

namespace hexapod { 

    namespace config {
        // all below definition use unit: mm
        const float kLegMountLeftRightX = 34.7;
        const float kLegMountOtherX = 25;
        const float kLegMountOtherY = 58;

        // ---- quadruped mounting (NodeQuadMini) ----
        // 四足没有 left/right 专用安装点，四条腿统一用 (±X, ±Y) 的安装点
        // 说明：腿部尺寸/连杆参数与六足共用，仅安装位置不同
        const float kQuadLegMountOtherX = 25.0;
        const float kQuadLegMountOtherY = 45.0;
        
        const float kLegRootToJoint1 = 19.4;
        const float kLegJoint1ToJoint2 = 32;
        const float kLegJoint2ToJoint3 = 43.8;
        const float kLegJoint3ToTip = 90.05;


        // timing setting. unit: ms
        const int movementInterval = 20;
        const int movementSwitchDuration = 150;

        // speed control. range: 0.25 - 1.0 (1.0 is fastest)
        const float defaultSpeed = 0.5;
        const float minSpeed = 0.25;
        const float maxSpeed = 1.0;
    }

    // Speed level enumeration
    enum SpeedLevel {
        SPEED_SLOWEST = 0,   // 0.25x (极慢)
        SPEED_SLOW = 1,      // 0.33x (慢速)
        SPEED_MEDIUM = 2,    // 0.5x (中速, default)
        SPEED_FAST = 3       // 1.0x (快速)
    };

    // Speed level to multiplier mapping
    const float speedLevelMultipliers[] = {0.25, 0.33, 0.5, 1.0};

}