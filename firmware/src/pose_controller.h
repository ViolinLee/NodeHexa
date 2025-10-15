#pragma once

#include "config.h"

#ifdef USE_REALTIME_GAIT

#include "base.h"
#include "gait_parameters.h"

namespace hexapod {

    // 姿态控制器
    class PoseController {
    public:
        PoseController();

        // 设置机身姿态
        void setBodyPose(const BodyPose& pose);
        
        // 应用姿态变换到足端位置
        // basePositions: 基础站立位置
        // 返回: 变换后的足端位置
        Locations applyPoseTransform(const Locations& basePositions);
        
        // 获取当前姿态
        const BodyPose& getBodyPose() const { return bodyPose_; }
        
    private:
        // 坐标变换：将点绕XYZ轴旋转
        void rotateX(const Point3D& in, float angleDeg, Point3D& out);
        void rotateY(const Point3D& in, float angleDeg, Point3D& out);
        void rotateZ(const Point3D& in, float angleDeg, Point3D& out);
        
        // 应用完整的姿态变换
        void transformPoint(const Point3D& in, int legIndex, Point3D& out);
        
        // 获取腿部安装位置
        void getLegMountPosition(int legIndex, Point3D& mountPos);
        
    private:
        BodyPose bodyPose_;     // 当前机身姿态
    };

} // namespace hexapod

#endif // USE_REALTIME_GAIT

