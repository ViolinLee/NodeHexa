#ifdef ROBOT_MODEL_NODEQUADMINI

#include <Arduino.h>

#include "quad_leg.h"
#include "config.h"
#include "debug.h"

#include <cmath>
#include "leg_kinematics.h"

using namespace hexapod::config;

namespace quadruped {

    namespace {

        // 旋转矩阵：
        //  x' = x * cos - y * sin
        //  y' = x * sin + y * cos

        #define SIN45   0.7071
        #define COS45   0.7071

        void rotate0(const hexapod::Point3D& src, hexapod::Point3D& dest) {
            dest = src;
        }

        void rotate45(const hexapod::Point3D& src, hexapod::Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * COS45;
            dest.z_ = src.z_;
        }

        void rotate135(const hexapod::Point3D& src, hexapod::Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate225(const hexapod::Point3D& src, hexapod::Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate315(const hexapod::Point3D& src, hexapod::Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * COS45;
            dest.z_ = src.z_;
        }


    }

    //
    // Public
    //

    Leg::Leg(int legIndex) : index_(legIndex) {
        // 四足腿定义（与六足保持一致的世界坐标系）：
        // - 世界坐标：X 向右为正，Y 向前为正；角度定义为 CCW：
        //      0° = +X, 90° = +Y
        // - 腿索引从 FR 开始，按顺时针（CW）编号：
        //      0=FR(45°), 1=BR(315°), 2=BL(225°), 3=FL(135°)
        //
        // 旋转约定与六足一致：
        // - world -> local : rotate(-theta)
        // - local -> world : rotate(+theta)

        switch (legIndex) {
        case 0: // FR, +45°
            mountPosition_ = hexapod::Point3D(kQuadLegMountOtherX, kQuadLegMountOtherY, 0);
            localConv_ = rotate315;
            worldConv_ = rotate45;
            break;
        case 1: // BR, 315° (-45°)
            mountPosition_ = hexapod::Point3D(kQuadLegMountOtherX, -kQuadLegMountOtherY, 0);
            localConv_ = rotate45;
            worldConv_ = rotate315;
            break;
        case 2: // BL, 225° (-135°)
            mountPosition_ = hexapod::Point3D(-kQuadLegMountOtherX, -kQuadLegMountOtherY, 0);
            localConv_ = rotate135;
            worldConv_ = rotate225;
            break;
        case 3: // FL, 135°
            mountPosition_ = hexapod::Point3D(-kQuadLegMountOtherX, kQuadLegMountOtherY, 0);
            localConv_ = rotate225;
            worldConv_ = rotate135;
            break;
        default:
            mountPosition_ = hexapod::Point3D(0, 0, 0);
            localConv_ = rotate0;
            worldConv_ = rotate0;
            break;
        }

        for (int i = 0; i < 3; i++)
            servos_[i] = new ServoQuad(legIndex, i);

        // 避免首次 moveTip() 用未初始化的 tipPos_ 做相等比较
        tipPos_ = hexapod::Point3D(0, 0, 0);
        tipPosLocal_ = hexapod::Point3D(0, 0, 0);
    }

    Leg::~Leg() {
        for (int i = 0; i < 3; i++) {
            delete servos_[i];
            servos_[i] = nullptr;
        }
    }

    void Leg::translateToLocal(const hexapod::Point3D& world, hexapod::Point3D& local) {
        localConv_(world - mountPosition_, local);
    }

    void Leg::translateToWorld(const hexapod::Point3D& local, hexapod::Point3D& world) {
        worldConv_(local, world);
        // The legacy world->local matrix uses rounded 0.7071 coefficients.
        // Divide by its squared norm so the reverse is its actual inverse.
        if (true) {
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

#endif // ROBOT_MODEL_NODEQUADMINI
