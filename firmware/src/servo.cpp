#include "servo.h"
#include "debug.h"

#include "pwm.h"

namespace hexapod { 

    namespace {
        // freq = 50, 1 tick = 1000000 / (50*4096) ~= 5us
        const static int kFrequency = 50;
        const static int kTickUs = 5;
        const static int kServoMiddle = 1500;
        const static int kServoMax = 2500;
        const static int kServoMin = 500;
        const static int kServoRange = kServoMax-kServoMiddle;

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

        int pwm2hexapod(int pwm) {
            if(pwm >= 16+8)
                return 9 + pwm - (16+8);
            if(pwm >= 16+5)
                return 15 + pwm - (16+5);
            if(pwm >= 16+2)
                return 12 + pwm - (16+2);
            if(pwm >= 8)
                return 6 + pwm - (8);
            if(pwm >= 5)
                return 0 + pwm - (5);
            if(pwm >= 2)
                return 3 + pwm - (2);
            return -1;
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

    void Servo::setAngle(float angle) {
        initPWM();

        if (angle > range_ + adjust_angle_) {
            LOG_INFO("exceed[%d][%f]", pwm2hexapod(pwmIndex_), angle);
            angle = range_;
        }
        else if(angle < -range_ + adjust_angle_) {
            LOG_INFO("exceed[%d][%f]", pwm2hexapod(pwmIndex_), angle);
            angle = -range_;
        }

        angle_ = angle;

        hal::PCA9685* pwm;
        int idx;
        if (pwmIndex_ < 16) {
            pwm = &pwmRight;
            idx = pwmIndex_;
        }
        else {
            pwm = &pwmLeft;
            idx = pwmIndex_ - 16;
        }
        angle -= adjust_angle_;
        if (inverse_)
            angle = -angle;

        int us = kServoMiddle + (angle + offset_)*(kServoRange/90);
        if (us > kServoMax)
            us = kServoMax;
        else if(us < kServoMin)
            us = kServoMin;

        pwm->setPWM(idx, 0, us/kTickUs);
        LOG_DEBUG("setAngle(%.2f, %d)", angle, us);
    }

    float Servo::getAngle(void) {
        return angle_;
    }

}