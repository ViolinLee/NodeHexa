
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "pwm.h"

namespace hexapod { namespace hal {

    PCA9685::PCA9685(int i2cAddress) : address_(static_cast<uint8_t>(i2cAddress)) {
        obj_ = (void*)new Adafruit_PWMServoDriver(i2cAddress);
    }

    PCA9685::~PCA9685() {
        delete ((Adafruit_PWMServoDriver*)obj_);
    }

    bool PCA9685::begin() {
        return ((Adafruit_PWMServoDriver*)obj_)->begin();
    }

    void PCA9685::setPWMFreq(int freq) {
        ((Adafruit_PWMServoDriver*)obj_)->setPWMFreq(freq);
    }

    bool PCA9685::setPWM(int index, int on, int off) {
        return ((Adafruit_PWMServoDriver*)obj_)->setPWM(
            index,
            static_cast<uint16_t>(on),
            static_cast<uint16_t>(off)
        ) == 0;
    }

    bool PCA9685::fullOff() {
        // PCA9685 ALL_LED_OFF_H: bit 4 forces every channel off. A single
        // register write avoids depending on MODE1 auto-increment state.
        Wire.beginTransmission(address_);
        Wire.write(static_cast<uint8_t>(0xFD));
        Wire.write(static_cast<uint8_t>(0x10));
        return Wire.endTransmission() == 0;
    }

    uint8_t PCA9685::address() const {
        return address_;
    }

}}
