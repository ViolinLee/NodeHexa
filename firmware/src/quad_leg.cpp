#include <Arduino.h>

#include "quad_leg.h"
#include "config.h"
#include "debug.h"

#include <cmath>

using namespace hexapod::config;

namespace quad {

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
        // 四足腿的空间分布：
        //  1 (FL)      0 (FR)
        //  2 (BL)      3 (BR)
        //
        // 其中位置与旋转近似参考六足中的 0,2,5,3 四条腿

        switch (legIndex) {
        case 0: // Front Right，近似六足 leg2 (-45/315 度)
            mountPosition_ = hexapod::Point3D(kLegMountOtherX, -kLegMountOtherY, 0);
            localConv_ = rotate45;
            worldConv_ = rotate315;
            break;
        case 1: // Front Left，近似六足 leg0 (+45 度)
            mountPosition_ = hexapod::Point3D(kLegMountOtherX, kLegMountOtherY, 0);
            localConv_ = rotate315;
            worldConv_ = rotate45;
            break;
        case 2: // Back Left，近似六足 leg5 (135 度)
            mountPosition_ = hexapod::Point3D(-kLegMountOtherX, kLegMountOtherY, 0);
            localConv_ = rotate225;
            worldConv_ = rotate135;
            break;
        case 3: // Back Right，近似六足 leg3 (-135/225 度)
            mountPosition_ = hexapod::Point3D(-kLegMountOtherX, -kLegMountOtherY, 0);
            localConv_ = rotate135;
            worldConv_ = rotate225;
            break;
        default:
            mountPosition_ = hexapod::Point3D(0, 0, 0);
            localConv_ = rotate0;
            worldConv_ = rotate0;
            break;
        }

        for (int i = 0; i < 3; i++)
            servos_[i] = new ServoQuad(legIndex, i);
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
        world += mountPosition_;
    }

    void Leg::setJointAngle(float angle[3]) {
        hexapod::Point3D to;
        _forwardKinematics(angle, to);
        moveTipLocal(to);
    }

    void Leg::moveTip(const hexapod::Point3D& to) {
        if (to == tipPos_)
            return;

        hexapod::Point3D local;
        translateToLocal(to, local);
        LOG_DEBUG("quad leg(%d) moveTip(%f,%f,%f)(%f,%f,%f)",
                  index_, to.x_, to.y_, to.z_, local.x_, local.y_, local.z_);
        _move(local);
        tipPos_ = to;
        tipPosLocal_ = local;
    }

    const hexapod::Point3D& Leg::getTipPosition(void) {
        return tipPos_;
    }

    void Leg::moveTipLocal(const hexapod::Point3D& to) {
        if (to == tipPosLocal_)
            return;

        hexapod::Point3D world;
        translateToWorld(to, world);
        _move(to);
        tipPos_ = world;
        tipPosLocal_ = to;
    }

    const hexapod::Point3D& Leg::getTipPositionLocal(void) {
        return tipPosLocal_;
    }

    //
    // Private: Kinematics
    //

    const float pi = std::acos(-1.0f);
    const float hpi = pi / 2.0f;

    void Leg::_forwardKinematics(float angle[3], hexapod::Point3D& out) {
        float radian[3];
        for (int i = 0; i < 3; i++)
            radian[i] = pi * angle[i] / 180.0f;

        float x = kLegJoint1ToJoint2 +
                  std::cos(radian[1]) * kLegJoint2ToJoint3 +
                  std::cos(radian[1] + radian[2] - hpi) * kLegJoint3ToTip;

        out.x_ = kLegRootToJoint1 + std::cos(radian[0]) * x;
        out.y_ = std::sin(radian[0]) * x;
        out.z_ = std::sin(radian[1]) * kLegJoint2ToJoint3 +
                 std::sin(radian[1] + radian[2] - hpi) * kLegJoint3ToTip;
    }

    void Leg::_inverseKinematics(const hexapod::Point3D& to, float angles[3]) {
        float x = to.x_ - kLegRootToJoint1;
        float y = to.y_;

        angles[0] = std::atan2(y, x) * 180.0f / pi;

        x = std::sqrt(x * x + y * y) - kLegJoint1ToJoint2;
        y = to.z_;
        float ar = std::atan2(y, x);
        float lr2 = x * x + y * y;
        float lr = std::sqrt(lr2);
        float a1 = std::acos((lr2 + kLegJoint2ToJoint3 * kLegJoint2ToJoint3 -
                              kLegJoint3ToTip * kLegJoint3ToTip) /
                             (2 * kLegJoint2ToJoint3 * lr));
        float a2 = std::acos((lr2 - kLegJoint2ToJoint3 * kLegJoint2ToJoint3 +
                              kLegJoint3ToTip * kLegJoint3ToTip) /
                             (2 * kLegJoint3ToTip * lr));
        angles[1] = (ar + a1) * 180.0f / pi;
        angles[2] = 90.0f - ((a1 + a2) * 180.0f / pi);
    }

    void Leg::_move(const hexapod::Point3D& to) {
        float angles[3];
        _inverseKinematics(to, angles);
        LOG_DEBUG("quad leg(%d) move: (%f,%f,%f)", index_, angles[0], angles[1], angles[2]);
        for (int i = 0; i < 3; i++) {
            servos_[i]->setAngle(angles[i]);
        }
    }

} // namespace quad

