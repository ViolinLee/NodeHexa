#pragma once

#include "base.h"
#include "servo_quad.h"

namespace quad {

    class Leg {
    public:
        explicit Leg(int legIndex);
        virtual ~Leg();

        // 坐标系转换：世界坐标 → 本地坐标
        void translateToLocal(const hexapod::Point3D& world, hexapod::Point3D& local);
        void translateToWorld(const hexapod::Point3D& local, hexapod::Point3D& world);

        void setJointAngle(float angle[3]);

        // 使用世界坐标控制足端
        void moveTip(const hexapod::Point3D& to);
        const hexapod::Point3D& getTipPosition(void);

        // 使用本地坐标控制足端
        void moveTipLocal(const hexapod::Point3D& to);
        const hexapod::Point3D& getTipPositionLocal(void);

        // 获取单个关节舵机
        ServoQuad* get(int partIndex) {
            return servos_[partIndex];
        }

        // 强制重置记录的足端位置
        void forceResetTipPosition() {
            tipPos_ = hexapod::Point3D(0, 0, 0);
            tipPosLocal_ = hexapod::Point3D(0, 0, 0);
        }

    private:
        // 本地坐标系下的正向 / 逆向运动学
        static void _forwardKinematics(float angle[3], hexapod::Point3D& out);
        static void _inverseKinematics(const hexapod::Point3D& to, float angles[3]);
        void _move(const hexapod::Point3D& to);

    private:
        int index_{0};
        ServoQuad* servos_[3]{};
        hexapod::Point3D mountPosition_;
        std::function<void(const hexapod::Point3D& src, hexapod::Point3D& dest)> localConv_;
        std::function<void(const hexapod::Point3D& src, hexapod::Point3D& dest)> worldConv_;

        hexapod::Point3D tipPos_;
        hexapod::Point3D tipPosLocal_;
    };

} // namespace quad

