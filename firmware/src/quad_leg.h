#pragma once

#include <functional>
#include "base.h"
#include "quad_servo.h"

namespace quadruped {

    class Leg {
    public:
        explicit Leg(int legIndex);
        virtual ~Leg();

        bool solveLocal(const hexapod::Point3D& tip, float angles[3]) const;
        bool canMoveTip(const hexapod::Point3D& world);
        bool hasPosition() const;

        // Coordinate transforms
        void translateToLocal(const hexapod::Point3D& world, hexapod::Point3D& local);
        void translateToWorld(const hexapod::Point3D& local, hexapod::Point3D& world);

        bool setJointAngle(float angle[3]);

        // 使用世界坐标控制足端
        bool moveTip(const hexapod::Point3D& to);
        const hexapod::Point3D& getTipPosition(void);

        // 使用本地坐标控制足端
        bool moveTipLocal(const hexapod::Point3D& to);
        const hexapod::Point3D& getTipPositionLocal(void);

        // 获取单个关节舵机
        ServoQuad* get(int partIndex) {
            return servos_[partIndex];
        }

        // 强制重置记录的足端位置
        void forceResetTipPosition() {
            for (int j = 0; j < 3; ++j) servos_[j]->invalidateCommand();
            tipPos_ = hexapod::Point3D(0, 0, 0);
            tipPosLocal_ = hexapod::Point3D(0, 0, 0);
        }

    private:
        int index_{0};
        ServoQuad* servos_[3]{};
        hexapod::Point3D mountPosition_;
        std::function<void(const hexapod::Point3D& src, hexapod::Point3D& dest)> localConv_;
        std::function<void(const hexapod::Point3D& src, hexapod::Point3D& dest)> worldConv_;

        hexapod::Point3D tipPos_;
        hexapod::Point3D tipPosLocal_;
    };

} // namespace quadruped

