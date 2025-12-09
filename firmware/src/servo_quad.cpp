#include "servo_quad.h"
#include "debug.h"
#include "pwm.h"

namespace quad {

    namespace {
        // freq = 50, 1 tick = 1000000 / (50*4096) ~= 5us
        const static int kFrequency = 50;
        const static int kTickUs = 5;
        const static int kServoMiddle = 1500;
        const static int kServoMax = 2500;
        const static int kServoMin = 500;
        const static int kServoRange = kServoMax - kServoMiddle;

        // 单片 PCA9685，地址固定为 0x40
        static hexapod::hal::PCA9685 pwm(0x40);
        static bool pwmInited = false;

        void initPWM() {
            if (pwmInited)
                return;

            pwm.begin();
            pwm.setPWMFreq(kFrequency);
            pwmInited = true;
        }

        // 简单的通道映射：legIndex * 3 + partIndex → 0..11
        int quad2pwm(int legIndex, int partIndex) {
            return legIndex * 3 + partIndex;
        }
    }

    void ServoQuad::init(void) {
        initPWM();
    }

    ServoQuad::ServoQuad(int legIndex, int partIndex) {
        pwmIndex_ = quad2pwm(legIndex, partIndex);
        inverse_ = (partIndex == 1);        // 与六足保持一致：第二关节反向
        adjust_angle_ = (partIndex == 1) ? 15.0f : 0.0f;
        range_ = (partIndex == 0) ? 45 : 60;
        angle_ = 0.0f;
        offset_ = 0;
    }

    void ServoQuad::setAngle(float angle) {
        initPWM();

        if (angle > range_ + adjust_angle_) {
            LOG_INFO("Quad servo exceed[%d][%f]", pwmIndex_, angle);
            angle = static_cast<float>(range_);
        }
        else if (angle < -range_ + adjust_angle_) {
            LOG_INFO("Quad servo exceed[%d][%f]", pwmIndex_, angle);
            angle = static_cast<float>(-range_);
        }

        angle_ = angle;

        int idx = pwmIndex_;
        angle -= adjust_angle_;
        if (inverse_)
            angle = -angle;

        int us = kServoMiddle + static_cast<int>((angle + offset_) * (kServoRange / 90.0f));
        if (us > kServoMax)
            us = kServoMax;
        else if (us < kServoMin)
            us = kServoMin;

        pwm.setPWM(idx, 0, us / kTickUs);
        LOG_DEBUG("Quad setAngle(%.2f, %d)", angle, us);
    }

    float ServoQuad::getAngle(void) {
        return angle_;
    }

} // namespace quad

