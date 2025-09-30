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

        // speed control. range: 0.3 - 1.0 (1.0 is fastest)
        const float defaultSpeed = 0.5;
        const float minSpeed = 0.3;
        const float maxSpeed = 1.0;
    }

    // Speed level enumeration
    enum SpeedLevel {
        SPEED_SLOW = 0,      // 0.3x
        SPEED_MEDIUM = 1,    // 0.5x (default)
        SPEED_FAST = 2,      // 0.75x
        SPEED_FASTEST = 3    // 1.0x
    };

    // Speed level to multiplier mapping
    const float speedLevelMultipliers[] = {0.3, 0.5, 0.75, 1.0};

}