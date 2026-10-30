#include <Arduino.h>

#include "leg.h"
#include "config.h"
#include "debug.h"

#include <cmath>
#include "leg_kinematics.h"

using namespace hexapod::config;

// Angle (CCW)
//          0 : + X-axis
//         90 : + Y-axis
//        180 : - X-axis
//        270 : - Y-axis

namespace hexapod {

    namespace {

        // rotate matrix:
        //  x' = x * cos - y * sin
        //  y' = x * sin + y * cos

        #define SIN45   0.7071
        #define COS45   0.7071

        void rotate0(const Point3D& src, Point3D& dest) {
            dest = src;
        }

        void rotate45(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * COS45;
            dest.z_ = src.z_;
        }

        void rotate135(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate180(const Point3D& src, Point3D& dest) {
            dest.x_ = -src.x_;
            dest.y_ = -src.y_;
            dest.z_ = src.z_;
        }

        void rotate225(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate315(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * COS45;
            dest.z_ = src.z_;
        }


    }

    // Public

    Leg::Leg(int legIndex): index_(legIndex) {
        switch(legIndex) {
        case 0: // 45 degree
            mountPosition_ = Point3D(kLegMountOtherX, kLegMountOtherY, 0);
            localConv_ = rotate315;
            worldConv_ = rotate45;
            break;
        case 1: // 0 degree
            mountPosition_ = Point3D(kLegMountLeftRightX, 0, 0);
            localConv_ = rotate0;
            worldConv_ = rotate0;
            break;
        case 2: // -45 or 315 degree
            mountPosition_ = Point3D(kLegMountOtherX, -kLegMountOtherY, 0);
            localConv_ = rotate45;
            worldConv_ = rotate315;
            break;
        case 3: // -135 or 225 degree
            mountPosition_ = Point3D(-kLegMountOtherX, -kLegMountOtherY, 0);
            localConv_ = rotate135;
            worldConv_ = rotate225;
            break;
        case 4: // 180 degree
            mountPosition_ = Point3D(-kLegMountLeftRightX, 0, 0);
            localConv_ = rotate180;
            worldConv_ = rotate180;
            break;
        case 5: // 135 degree
            mountPosition_ = Point3D(-kLegMountOtherX, kLegMountOtherY, 0);
            localConv_ = rotate225;
            worldConv_ = rotate135;
            break;
        default:
            mountPosition_ = Point3D(0, 0, 0);
            localConv_ = rotate0;
            worldConv_ = rotate0;
            break;
        }

        for(int i=0;i<3;i++)
            servos_[i] = new hexapod::Servo(legIndex, i);

        // 避免首次 moveTip() 用未初始化的 tipPos_ 做相等比较
        tipPos_ = Point3D(0, 0, 0);
        tipPosLocal_ = Point3D(0, 0, 0);
    }

    Leg::~Leg() {
        for(int i=0;i<3;i++) {
            delete servos_[i];
            servos_[i] = nullptr;
        }
    }

    void Leg::translateToLocal(const Point3D& world, Point3D& local) {
        localConv_(world - mountPosition_, local);
    }

    void Leg::translateToWorld(const Point3D& local, Point3D& world) {
        worldConv_(local, world);
        // The legacy world->local matrix uses rounded 0.7071 coefficients.
        // Divide by its squared norm so the reverse is its actual inverse.
        if (index_ != 1 && index_ != 4) {
            world.x_ /= 2.0f * 0.7071f * 0.7071f;
            world.y_ /= 2.0f * 0.7071f * 0.7071f;
        }
        world += mountPosition_;
    }

    bool Leg::hasPosition() const {
        return servos_[0]->hasCommand() && servos_[1]->hasCommand() && servos_[2]->hasCommand();
    }

    bool Leg::solveLocal(const hexapod::Point3D& tip, float angles[3]) const {
        if (!motioncore::inverse(tip, angles)) return false;
        for (int j = 0; j < 3; ++j) if (!servos_[j]->canSetAngle(angles[j])) return false;
        return true;
    }

    bool Leg::canMoveTip(const hexapod::Point3D& world) {
        hexapod::Point3D local;
        translateToLocal(world, local);
        float angles[3];
        return solveLocal(local, angles);
    }

    bool Leg::setJointAngle(float angles[3]) {
        for (int j = 0; j < 3; ++j) if (!servos_[j]->canSetAngle(angles[j])) return false;
        bool ok = true;
        for (int j = 0; j < 3; ++j) {
            if (!servos_[j]->setAngle(angles[j])) { ok = false; break; }
        }
        // I2C writes are not atomic. On failure retain the last successful
        // command for each joint, rather than recording the requested endpoint.
        if (hasPosition()) {
            float applied[3];
            for (int j = 0; j < 3; ++j) applied[j] = servos_[j]->getAngle();
            motioncore::forward(applied, tipPosLocal_);
            translateToWorld(tipPosLocal_, tipPos_);
        }
        return ok;
    }

    bool Leg::moveTip(const hexapod::Point3D& to) {
        hexapod::Point3D local;
        translateToLocal(to, local);
        float angles[3];
        if (!solveLocal(local, angles) || !setJointAngle(angles)) return false;
        tipPos_ = to;
        tipPosLocal_ = local;
        return true;
    }

    const hexapod::Point3D& Leg::getTipPosition() { return tipPos_; }

    bool Leg::moveTipLocal(const hexapod::Point3D& to) {
        float angles[3];
        if (!solveLocal(to, angles) || !setJointAngle(angles)) return false;
        tipPosLocal_ = to;
        translateToWorld(to, tipPos_);
        return true;
    }

    const hexapod::Point3D& Leg::getTipPositionLocal() { return tipPosLocal_; }

} // namespace
