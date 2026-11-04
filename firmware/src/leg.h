#pragma once

#include "base.h"
#include "servo.h"

#include <functional>

namespace hexapod {

    class Leg {
    public:
        Leg(int legIndex);
        virtual ~Leg();

        bool solveLocal(const Point3D& tip, float angles[3]) const;
        bool canMoveTip(const Point3D& world);
        bool hasPosition() const;

        // Coordinate transforms
        void translateToLocal(const Point3D& world, Point3D& local);
        void translateToWorld(const Point3D& local, Point3D& world);

        bool setJointAngle(float angle[3]);

        // world coordinate system (default)
        bool moveTip(const Point3D& to);
        const Point3D& getTipPosition(void);

        // local coordinate system version
        bool moveTipLocal(const Point3D& to);
        const Point3D& getTipPositionLocal(void);

        // 
        Servo* get(int partIndex) {
            return servos_[partIndex];
        }

        //
        void forceResetTipPosition() {
            for (int j = 0; j < 3; ++j) servos_[j]->invalidateCommand();
            tipPos_ = Point3D(0, 0, 0);
            tipPosLocal_ = Point3D(0, 0, 0);
        }

    private:
        int index_;
        Servo* servos_[3];
        Point3D mountPosition_;
        std::function<void(const Point3D& src, Point3D& dest)> localConv_;
        std::function<void(const Point3D& src, Point3D& dest)> worldConv_;

        Point3D tipPos_;
        Point3D tipPosLocal_;

    };

}