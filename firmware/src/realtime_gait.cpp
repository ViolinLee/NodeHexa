#include "realtime_gait.h"

#ifdef USE_REALTIME_GAIT

#include "debug.h"
#include "standby_position.h"
#include <Arduino.h>
#include <cmath>

namespace hexapod {

    namespace {
        const float PI = 3.14159265359;
        
        // 计算六足机器人腿部中心到机身中心的距离和角度
        struct LegMountInfo {
            float radius;   // 距离
            float angle;    // 角度（弧度）
        };
        
        // 六条腿的安装信息
        const LegMountInfo kLegMountInfo[6] = {
            {std::sqrt(config::kLegMountOtherX * config::kLegMountOtherX + 
                      config::kLegMountOtherY * config::kLegMountOtherY), 
             std::atan2(config::kLegMountOtherY, config::kLegMountOtherX)},    // leg0: 前右 (45度)
            
            {config::kLegMountLeftRightX, 0.0},                                  // leg1: 右 (0度)
            
            {std::sqrt(config::kLegMountOtherX * config::kLegMountOtherX + 
                      config::kLegMountOtherY * config::kLegMountOtherY), 
             std::atan2(-config::kLegMountOtherY, config::kLegMountOtherX)},   // leg2: 后右 (-45度)
            
            {std::sqrt(config::kLegMountOtherX * config::kLegMountOtherX + 
                      config::kLegMountOtherY * config::kLegMountOtherY), 
             std::atan2(-config::kLegMountOtherY, -config::kLegMountOtherX)},  // leg3: 后左 (-135度)
            
            {config::kLegMountLeftRightX, PI},                                   // leg4: 左 (180度)
            
            {std::sqrt(config::kLegMountOtherX * config::kLegMountOtherX + 
                      config::kLegMountOtherY * config::kLegMountOtherY), 
             std::atan2(config::kLegMountOtherY, -config::kLegMountOtherX)}     // leg5: 前左 (135度)
        };
    }

    RealtimeGait::RealtimeGait():
        gaitParams_(),
        velocity_(),
        position_(),
        startTime_(0),
        currentPhase_(0.0)
    {
    }

    void RealtimeGait::setGaitParameters(const GaitParameters& params) {
        gaitParams_ = params;
        gaitParams_.validate();
        
        LOG_INFO("步态参数更新: stride=%.1f, height=%.1f, period=%.1f", 
                 gaitParams_.stride, gaitParams_.liftHeight, gaitParams_.period);
    }

    void RealtimeGait::setVelocity(const Velocity& vel) {
        velocity_ = vel;
        velocity_.validate();
        
        LOG_DEBUG("速度更新: vx=%.1f, vy=%.1f, vyaw=%.1f", 
                  velocity_.vx, velocity_.vy, velocity_.vyaw);
    }

    void RealtimeGait::reset() {
        startTime_ = millis();
        currentPhase_ = 0.0;
        
        // 重置到站立位置
        position_ = standby::getStandbyLocations();
        
        LOG_INFO("步态已重置");
    }

    const Locations& RealtimeGait::update(int elapsed) {
        // 更新相位
        if (startTime_ == 0) {
            startTime_ = millis();
            position_ = standby::getStandbyLocations();
        }
        
        unsigned long currentTime = millis();
        float totalTime = (currentTime - startTime_);
        currentPhase_ = fmod(totalTime / gaitParams_.period, 1.0);
        
        // 如果速度为零，保持站立姿态
        if (velocity_.isZero()) {
            position_ = standby::getStandbyLocations();
            return position_;
        }
        
        // 获取基础站立位置
        Locations basePositions = standby::getStandbyLocations();
        
        // 计算移动方向（根据vx, vy计算角度）
        float moveAngle = 0.0;
        float moveSpeed = 0.0;
        
        if (velocity_.vx != 0.0 || velocity_.vy != 0.0) {
            moveSpeed = std::sqrt(velocity_.vx * velocity_.vx + velocity_.vy * velocity_.vy);
            moveAngle = std::atan2(velocity_.vy, velocity_.vx); // 弧度
        }
        
        // 计算每条腿的位置
        for (int i = 0; i < 6; i++) {
            // 获取基础位置
            Point3D legPos = basePositions[i];
            
            // 计算当前腿的相位
            float legPhase = fmod(currentPhase_ + kTrotPhaseOffset[i], 1.0);
            
            // 应用移动轨迹
            if (moveSpeed > 0.0) {
                // 根据速度缩放步幅
                float strideScale = moveSpeed / config::maxVelocityX;
                if (strideScale > 1.0) strideScale = 1.0;
                float actualStride = gaitParams_.stride * strideScale;
                
                Point3D moveOffset;
                
                // 判断支撑相还是摆动相
                if (legPhase < gaitParams_.dutyFactor) {
                    // 支撑相
                    float stanceRatio = legPhase / gaitParams_.dutyFactor;
                    calculateStancePosition(stanceRatio, actualStride, moveAngle, moveOffset);
                } else {
                    // 摆动相
                    float swingRatio = (legPhase - gaitParams_.dutyFactor) / (1.0 - gaitParams_.dutyFactor);
                    calculateSwingPosition(swingRatio, actualStride, moveAngle, 
                                          gaitParams_.liftHeight, moveOffset);
                }
                
                legPos += moveOffset;
            }
            
            // 应用旋转偏移
            if (velocity_.vyaw != 0.0) {
                Point3D rotOffset;
                applyRotation(i, velocity_.vyaw, rotOffset);
                legPos += rotOffset;
            }
            
            // 更新位置（使用新的setter）
            position_[i] = legPos;
        }
        
        return position_;
    }

    void RealtimeGait::calculateStancePosition(float phaseRatio, float stride, 
                                               float angle, Point3D& position) {
        // 支撑相：从后向前线性移动
        // phaseRatio: 0 -> 1 (从后到前)
        float offset = stride * (0.5 - phaseRatio); // [-stride/2, stride/2]
        
        position.x_ = offset * std::cos(angle);
        position.y_ = offset * std::sin(angle);
        position.z_ = 0.0;
    }

    void RealtimeGait::calculateSwingPosition(float phaseRatio, float stride, 
                                              float angle, float liftHeight, 
                                              Point3D& position) {
        // 摆动相：半圆形轨迹
        // phaseRatio: 0 -> 1
        
        // X-Y平面：从-stride/2 到 +stride/2
        float offset = stride * (phaseRatio - 0.5);
        
        // Z轴：半圆形抬起
        float zOffset = liftHeight * std::sin(PI * phaseRatio);
        
        position.x_ = offset * std::cos(angle);
        position.y_ = offset * std::sin(angle);
        position.z_ = zOffset;
    }

    void RealtimeGait::applyRotation(int legIndex, float vyaw, Point3D& offset) {
        // 根据偏航角速度计算旋转偏移
        // vyaw单位: 度/s
        
        const LegMountInfo& mountInfo = kLegMountInfo[legIndex];
        
        // 计算旋转角度（每个周期的旋转量）
        float rotAngle = vyaw * gaitParams_.period / 1000.0; // 度
        float rotAngleRad = rotAngle * PI / 180.0; // 弧度
        
        // 计算旋转导致的位置偏移
        float radius = mountInfo.radius;
        float currentAngle = mountInfo.angle;
        
        // 圆周运动的切线方向
        float tangentX = -std::sin(currentAngle);
        float tangentY = std::cos(currentAngle);
        
        // 旋转偏移
        float arcLength = radius * rotAngleRad;
        
        offset.x_ = tangentX * arcLength * 0.5;
        offset.y_ = tangentY * arcLength * 0.5;
        offset.z_ = 0.0;
    }

} // namespace hexapod

#endif // USE_REALTIME_GAIT

