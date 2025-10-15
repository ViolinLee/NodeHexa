#pragma once

//==============================================================================
// 运动模式编译开关
// 注意：同时只能启用一种模式
//==============================================================================
// #define USE_WIFI_CONTROL      // WiFi Web控制 + 预定义动作组模式
#define USE_BLE_CONTROL          // BLE遥控 + 实时步态模式

#ifdef USE_BLE_CONTROL
  #define USE_REALTIME_GAIT      // 启用实时步态
  #define USE_BLE_COMM           // 启用BLE通讯
#else
  #define USE_PREDEFINED_GAIT    // 使用预定义动作组
  #define USE_WIFI_COMM          // 使用WiFi通讯
#endif

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

#ifdef USE_REALTIME_GAIT
        // Realtime gait parameters
        const float defaultStride = 50.0;       // 默认步幅 (mm)
        const float minStride = 30.0;           // 最小步幅 (mm)
        const float maxStride = 80.0;           // 最大步幅 (mm)
        
        const float defaultLiftHeight = 25.0;   // 默认抬腿高度 (mm)
        const float minLiftHeight = 15.0;       // 最小抬腿高度 (mm)
        const float maxLiftHeight = 40.0;       // 最大抬腿高度 (mm)
        
        const float defaultGaitPeriod = 800.0;  // 默认步态周期 (ms)
        const float minGaitPeriod = 500.0;      // 最小步态周期 (ms)
        const float maxGaitPeriod = 1500.0;     // 最大步态周期 (ms)
        
        const float defaultDutyFactor = 0.5;    // 默认占空比
        const float minDutyFactor = 0.4;        // 最小占空比
        const float maxDutyFactor = 0.6;        // 最大占空比
        
        // Body pose parameters
        const float maxRoll = 30.0;             // 最大横滚角 (度)
        const float maxPitch = 30.0;            // 最大俯仰角 (度)
        const float maxYaw = 30.0;              // 最大偏航角 (度)
        const float maxHeightOffset = 50.0;     // 最大高度偏移 (mm)
        
        // Velocity parameters
        const float maxVelocityX = 200.0;       // 最大X速度 (mm/s)
        const float maxVelocityY = 200.0;       // 最大Y速度 (mm/s)
        const float maxVelocityYaw = 90.0;      // 最大偏航速度 (度/s)
#endif
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