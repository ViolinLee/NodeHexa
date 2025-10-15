#pragma once

#include "config.h"

namespace hexapod {

#ifdef USE_REALTIME_GAIT

    // 控制模式枚举
    enum ControlMode {
        MODE_STAND = 0,     // 站立模式（姿态控制）
        MODE_WALK = 1,      // 行走模式（步态控制）
        MODE_TRICK = 2      // 特技模式
    };

    // 特技动作枚举
    enum TrickAction {
        TRICK_NONE = 0,     // 无特技
        TRICK_A = 1,        // 后半身蹲抬前手
        TRICK_B = 2,        // 跳舞
        TRICK_C = 3,        // 预留
        TRICK_D = 4         // 预留
    };

    // 步态参数结构
    struct GaitParameters {
        float stride;           // 步幅 (mm)
        float liftHeight;       // 抬腿高度 (mm)
        float period;           // 步态周期 (ms)
        float dutyFactor;       // 占空比 [0-1]
        
        GaitParameters():
            stride(config::defaultStride),
            liftHeight(config::defaultLiftHeight),
            period(config::defaultGaitPeriod),
            dutyFactor(config::defaultDutyFactor)
        {}
        
        // 参数验证和限制
        void validate() {
            if (stride < config::minStride) stride = config::minStride;
            if (stride > config::maxStride) stride = config::maxStride;
            
            if (liftHeight < config::minLiftHeight) liftHeight = config::minLiftHeight;
            if (liftHeight > config::maxLiftHeight) liftHeight = config::maxLiftHeight;
            
            if (period < config::minGaitPeriod) period = config::minGaitPeriod;
            if (period > config::maxGaitPeriod) period = config::maxGaitPeriod;
            
            if (dutyFactor < config::minDutyFactor) dutyFactor = config::minDutyFactor;
            if (dutyFactor > config::maxDutyFactor) dutyFactor = config::maxDutyFactor;
        }
    };

    // 机身姿态结构
    struct BodyPose {
        float roll;     // 横滚角 (度)
        float pitch;    // 俯仰角 (度)
        float yaw;      // 偏航角 (度)
        float x;        // X位置偏移 (mm)
        float y;        // Y位置偏移 (mm)
        float z;        // Z位置偏移 (mm)
        
        BodyPose():
            roll(0.0), pitch(0.0), yaw(0.0),
            x(0.0), y(0.0), z(0.0)
        {}
        
        // 参数验证和限制
        void validate() {
            if (roll < -config::maxRoll) roll = -config::maxRoll;
            if (roll > config::maxRoll) roll = config::maxRoll;
            
            if (pitch < -config::maxPitch) pitch = -config::maxPitch;
            if (pitch > config::maxPitch) pitch = config::maxPitch;
            
            if (yaw < -config::maxYaw) yaw = -config::maxYaw;
            if (yaw > config::maxYaw) yaw = config::maxYaw;
            
            if (z < -config::maxHeightOffset) z = -config::maxHeightOffset;
            if (z > config::maxHeightOffset) z = config::maxHeightOffset;
        }
    };

    // 运动速度结构
    struct Velocity {
        float vx;       // X方向速度 (mm/s)
        float vy;       // Y方向速度 (mm/s)
        float vyaw;     // 偏航角速度 (度/s)
        
        Velocity():
            vx(0.0), vy(0.0), vyaw(0.0)
        {}
        
        // 参数验证和限制
        void validate() {
            if (vx < -config::maxVelocityX) vx = -config::maxVelocityX;
            if (vx > config::maxVelocityX) vx = config::maxVelocityX;
            
            if (vy < -config::maxVelocityY) vy = -config::maxVelocityY;
            if (vy > config::maxVelocityY) vy = config::maxVelocityY;
            
            if (vyaw < -config::maxVelocityYaw) vyaw = -config::maxVelocityYaw;
            if (vyaw > config::maxVelocityYaw) vyaw = config::maxVelocityYaw;
        }
        
        // 判断是否静止
        bool isZero() const {
            return (vx == 0.0 && vy == 0.0 && vyaw == 0.0);
        }
    };

    // Trot步态相位偏移（六足）
    // 腿序: 0-前右, 1-右, 2-后右, 3-后左, 4-左, 5-前左
    const float kTrotPhaseOffset[6] = {
        0.0,    // leg0: 第一组
        0.5,    // leg1: 第二组
        0.0,    // leg2: 第一组
        0.5,    // leg3: 第二组
        0.0,    // leg4: 第一组
        0.5     // leg5: 第二组
    };

#endif // USE_REALTIME_GAIT

}

