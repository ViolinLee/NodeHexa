#pragma once

namespace hexapod { 

    namespace config {
        // all below definition use unit: mm
        const float kLegMountLeftRightX = 29.87;
        const float kLegMountOtherX = 22.41;
        const float kLegMountOtherY = 55.41;
        
        const float kLegRootToJoint1 = 20.75;
        const float kLegJoint1ToJoint2 = 28.0;
        const float kLegJoint2ToJoint3 = 42.6;
        const float kLegJoint3ToTip = 89.07;


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