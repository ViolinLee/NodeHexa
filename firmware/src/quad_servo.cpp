#ifdef ROBOT_MODEL_NODEQUADMINI

#include "quad_servo.h"
#include "debug.h"
#include "pwm.h"
#include "servo_output.h"

namespace quadruped {

    namespace {
        // freq = 50, 1 tick = 1000000 / (50*4096) ~= 5us
        const static int kFrequency = 50;

        // 单片 PCA9685，地址固定为 0x40
        static hexapod::hal::PCA9685 pwm(0x40);
        static bool pwmInited = false;

        void initPWM() {
            if (pwmInited)
                return;

            pwm.begin();
            pwm.setPWMFreq(kFrequency);
            servo_output::registerController(&pwm);
            pwmInited = true;
        }

        // legIndex: 0=FR, 1=BR, 2=BL, 3=FL
        // partIndex: 0=关节1, 1=关节2, 2=关节3
        int quad2pwm(int legIndex, int partIndex) {
            if (legIndex < 0 || legIndex >= 4 || partIndex < 0 || partIndex >= 3)
                return 0;
            static constexpr int kMap[4][3] = {
                {13, 14, 15}, // FR
                {12, 11, 10}, // BR
                {3, 4, 5},    // BL
                {2, 1, 0},    // FL
            };
            return kMap[legIndex][partIndex];
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

    bool ServoQuad::setAngle(float angle) {
        int ticks;
        if (!mapping().pulse(angle, ticks)) return false;
        if (!servo_output::writeAllowed()) return false;
        if (commanded_ && angle == angle_) return true;
        initPWM();
        if (!servo_output::write(pwm, pwmIndex_, 0, ticks)) return false;
        angle_ = angle;
        commanded_ = true;
        return true;
    }

    float ServoQuad::getAngle(void) {
        return angle_;
    }

} // namespace quadruped

#endif // ROBOT_MODEL_NODEQUADMINI
