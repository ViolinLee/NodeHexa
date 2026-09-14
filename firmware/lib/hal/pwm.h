#pragma once

#include <stdint.h>

namespace hexapod { namespace hal {

    class PCA9685 {
    public:
        PCA9685(int i2cAddress = 0x40);
        ~PCA9685();

        bool begin();
        void setPWMFreq(int freq);
        bool setPWM(int index, int on, int off);
        bool fullOff();
        uint8_t address() const;

    private:
        void* obj_;
        uint8_t address_;
    };

}}
