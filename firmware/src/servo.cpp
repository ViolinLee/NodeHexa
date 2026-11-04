#include "servo.h"
#include "debug.h"

#include "pwm.h"
#include "servo_output.h"

namespace hexapod { 

    namespace {
        // freq = 50, 1 tick = 1000000 / (50*4096) ~= 5us
        const static int kFrequency = 50;

        auto pwmLeft = hal::PCA9685(0x41);  // 左侧拓展板需要焊接跳帽以设置I2C地址
        auto pwmRight = hal::PCA9685(0x40);
        bool pwmInited = false;
 
        void initPWM(void) {
            if (pwmInited)
                return;

            pwmLeft.begin();
            pwmLeft.setPWMFreq(kFrequency);
            pwmRight.begin();
            pwmRight.setPWMFreq(kFrequency);
            servo_output::registerController(&pwmLeft);
            servo_output::registerController(&pwmRight);
            pwmInited = true;
        }

        int hexapod2pwm(int legIndex, int partIndex) {
            switch(legIndex) {
            case 0:     return 5 + partIndex;
            case 1:     return 2 + partIndex;
            case 2:     return 8 + partIndex;
            case 3:     return 16 + 8 + partIndex;
            case 4:     return 16 + 2 + partIndex;
            case 5:     return 16 + 5 + partIndex;
            default:    return 0;
            }
        }


    }

    void Servo::init(void) {
        initPWM();
    }

    Servo::Servo(int legIndex, int partIndex) {
        pwmIndex_ = hexapod2pwm(legIndex, partIndex);
        inverse_ = partIndex == 1 ? true : false;
        adjust_angle_ = partIndex == 1 ? 15.0 : 0.0;
        range_ = partIndex == 0 ? 45 : 60;
        angle_ = 0;
        offset_ = 0;
    }

    bool Servo::setAngle(float angle) {
        int ticks;
        if (!mapping().pulse(angle, ticks)) return false;
        if (!servo_output::writeAllowed()) return false;
        if (commanded_ && angle == angle_) return true;
        initPWM();
        if (!servo_output::write((pwmIndex_ < 16 ? pwmRight : pwmLeft), pwmIndex_ % 16, 0, ticks)) return false;
        angle_ = angle;
        commanded_ = true;
        return true;
    }

    float Servo::getAngle(void) {
        return angle_;
    }

}
