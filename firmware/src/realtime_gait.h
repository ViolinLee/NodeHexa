#pragma once

#include "config.h"

#ifdef USE_REALTIME_GAIT

#include "base.h"
#include "gait_parameters.h"

namespace hexapod {

    // 实时步态生成器
    class RealtimeGait {
    public:
        RealtimeGait();

        // 设置步态参数
        void setGaitParameters(const GaitParameters& params);
        
        // 设置速度（立即生效）
        void setVelocity(const Velocity& vel);
        
        // 更新步态（每个控制周期调用）
        // elapsed: 距离上次调用的时间 (ms)
        // 返回: 六条腿的足端位置
        const Locations& update(int elapsed);
        
        // 重置步态（回到初始状态）
        void reset();
        
        // 获取当前参数
        const GaitParameters& getGaitParameters() const { return gaitParams_; }
        const Velocity& getVelocity() const { return velocity_; }
        
    private:
        // 计算支撑相位置
        void calculateStancePosition(float phaseRatio, float stride, float angle, Point3D& position);
        
        // 计算摆动相位置（半圆形轨迹）
        void calculateSwingPosition(float phaseRatio, float stride, float angle, float liftHeight, Point3D& position);
        
        // 计算旋转对位置的影响
        void applyRotation(int legIndex, float vyaw, Point3D& offset);
        
    private:
        GaitParameters gaitParams_;     // 步态参数
        Velocity velocity_;             // 当前速度
        Locations position_;            // 当前六条腿位置
        
        unsigned long startTime_;       // 步态开始时间 (ms)
        float currentPhase_;            // 当前相位 [0, 1)
    };

} // namespace hexapod

#endif // USE_REALTIME_GAIT

