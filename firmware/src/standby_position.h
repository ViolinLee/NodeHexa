#pragma once

#include "base.h"
#include "config.h"

namespace hexapod {

    // 站立位置计算辅助函数
    // 这些宏和计算从movements.cpp中提取，供实时步态使用
    
    namespace standby {
        const float SIN30 = 0.5;
        const float COS30 = 0.866;
        const float SIN45 = 0.7071;
        const float COS45 = 0.7071;
        const float SIN15 = 0.2588;
        const float COS15 = 0.9659;

        // 站立高度
        inline float getStandbyZ() {
            return config::kLegJoint3ToTip * COS15 - config::kLegJoint2ToJoint3 * SIN30;
        }

        // 左右腿X位置
        inline float getLeftRightX() {
            return config::kLegMountLeftRightX + config::kLegRootToJoint1 + 
                   config::kLegJoint1ToJoint2 + 
                   (config::kLegJoint2ToJoint3 * COS30) + 
                   config::kLegJoint3ToTip * SIN15;
        }

        // 其他腿X位置
        inline float getOtherX() {
            return config::kLegMountOtherX + 
                   (config::kLegRootToJoint1 + config::kLegJoint1ToJoint2 + 
                    (config::kLegJoint2ToJoint3 * COS30) + 
                    config::kLegJoint3ToTip * SIN15) * COS45;
        }

        // 其他腿Y位置
        inline float getOtherY() {
            return config::kLegMountOtherY + 
                   (config::kLegRootToJoint1 + config::kLegJoint1ToJoint2 + 
                    (config::kLegJoint2ToJoint3 * COS30) + 
                    config::kLegJoint3ToTip * SIN15) * SIN45;
        }

        // 获取站立位置（世界坐标系）
        inline Locations getStandbyLocations() {
            float standbyZ = getStandbyZ();
            float leftRightX = getLeftRightX();
            float otherX = getOtherX();
            float otherY = getOtherY();

            return Locations {
                {otherX, otherY, -standbyZ},      // leg0: 前右
                {leftRightX, 0, -standbyZ},       // leg1: 右
                {otherX, -otherY, -standbyZ},     // leg2: 后右
                {-otherX, -otherY, -standbyZ},    // leg3: 后左
                {-leftRightX, 0, -standbyZ},      // leg4: 左
                {-otherX, otherY, -standbyZ}      // leg5: 前左
            };
        }
    }

} // namespace hexapod

