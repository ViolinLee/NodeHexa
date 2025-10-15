#include "pose_controller.h"

#ifdef USE_REALTIME_GAIT

#include "debug.h"
#include "standby_position.h"
#include <cmath>

namespace hexapod {

    namespace {
        const float PI = 3.14159265359;
        const float DEG_TO_RAD = PI / 180.0;
        const float RAD_TO_DEG = 180.0 / PI;
    }

    PoseController::PoseController():
        bodyPose_()
    {
    }

    void PoseController::setBodyPose(const BodyPose& pose) {
        bodyPose_ = pose;
        bodyPose_.validate();
        
        LOG_DEBUG("姿态更新: roll=%.1f, pitch=%.1f, yaw=%.1f, z=%.1f", 
                  bodyPose_.roll, bodyPose_.pitch, bodyPose_.yaw, bodyPose_.z);
    }

    Locations PoseController::applyPoseTransform(const Locations& basePositions) {
        Locations transformed = basePositions;
        
        // 如果姿态为零，直接返回
        if (bodyPose_.roll == 0.0 && bodyPose_.pitch == 0.0 && 
            bodyPose_.yaw == 0.0 && bodyPose_.z == 0.0) {
            return transformed;
        }
        
        // 对每条腿应用姿态变换
        for (int i = 0; i < 6; i++) {
            const Point3D& basePos = basePositions[i];
            Point3D transformedPos;
            transformPoint(basePos, i, transformedPos);
            
            // 使用新的索引操作符更新位置
            transformed[i] = transformedPos;
        }
        
        return transformed;
    }

    void PoseController::rotateX(const Point3D& in, float angleDeg, Point3D& out) {
        float angleRad = angleDeg * DEG_TO_RAD;
        float cosA = std::cos(angleRad);
        float sinA = std::sin(angleRad);
        
        out.x_ = in.x_;
        out.y_ = in.y_ * cosA - in.z_ * sinA;
        out.z_ = in.y_ * sinA + in.z_ * cosA;
    }

    void PoseController::rotateY(const Point3D& in, float angleDeg, Point3D& out) {
        float angleRad = angleDeg * DEG_TO_RAD;
        float cosA = std::cos(angleRad);
        float sinA = std::sin(angleRad);
        
        out.x_ = in.x_ * cosA + in.z_ * sinA;
        out.y_ = in.y_;
        out.z_ = -in.x_ * sinA + in.z_ * cosA;
    }

    void PoseController::rotateZ(const Point3D& in, float angleDeg, Point3D& out) {
        float angleRad = angleDeg * DEG_TO_RAD;
        float cosA = std::cos(angleRad);
        float sinA = std::sin(angleRad);
        
        out.x_ = in.x_ * cosA - in.y_ * sinA;
        out.y_ = in.x_ * sinA + in.y_ * cosA;
        out.z_ = in.z_;
    }

    void PoseController::transformPoint(const Point3D& in, int legIndex, Point3D& out) {
        // 获取腿部安装位置
        Point3D mountPos;
        getLegMountPosition(legIndex, mountPos);
        
        // 1. 将足端位置转换到相对于机身中心的坐标
        Point3D relativePos = in - mountPos;
        
        // 2. 应用旋转变换（ZYX顺序）
        Point3D temp1, temp2, temp3;
        rotateZ(relativePos, bodyPose_.yaw, temp1);
        rotateY(temp1, bodyPose_.pitch, temp2);
        rotateX(temp2, bodyPose_.roll, temp3);
        
        // 3. 应用位置偏移
        temp3.x_ += bodyPose_.x;
        temp3.y_ += bodyPose_.y;
        temp3.z_ += bodyPose_.z;
        
        // 4. 转换回世界坐标
        out = temp3 + mountPos;
    }

    void PoseController::getLegMountPosition(int legIndex, Point3D& mountPos) {
        switch(legIndex) {
        case 0: // 前右
            mountPos = Point3D(config::kLegMountOtherX, config::kLegMountOtherY, 0);
            break;
        case 1: // 右
            mountPos = Point3D(config::kLegMountLeftRightX, 0, 0);
            break;
        case 2: // 后右
            mountPos = Point3D(config::kLegMountOtherX, -config::kLegMountOtherY, 0);
            break;
        case 3: // 后左
            mountPos = Point3D(-config::kLegMountOtherX, -config::kLegMountOtherY, 0);
            break;
        case 4: // 左
            mountPos = Point3D(-config::kLegMountLeftRightX, 0, 0);
            break;
        case 5: // 前左
            mountPos = Point3D(-config::kLegMountOtherX, config::kLegMountOtherY, 0);
            break;
        default:
            mountPos = Point3D(0, 0, 0);
            break;
        }
    }

} // namespace hexapod

#endif // USE_REALTIME_GAIT

