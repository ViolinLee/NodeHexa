#pragma once

#include "servo_mapping.h"

namespace hexapod { 

    class Servo {
    public:
        static void init(void);

    public:
        Servo(int legIndex, int partIndex);

        // angle: 0 means center, range is -60~60
        bool setAngle(float angle);
        motioncore::AngleLimits limits() const { return mapping().limits(); }
        bool canSetAngle(float angle) const { int ticks; return mapping().pulse(angle, ticks); }
        void invalidateCommand() { commanded_ = false; }
        bool hasCommand() const { return commanded_; }
        float getAngle(void);

        void getParameter(int& offset) {
            offset = offset_;
        }

        void setParameter(int offset, bool update = true) {
            offset_ = offset;
            commanded_ = false;
            if (update)
                setAngle(angle_);
        }

    private:
        motioncore::ServoMapping mapping() const {
            return {inverse_, adjust_angle_, static_cast<float>(range_), offset_, false};
        }
        bool commanded_ = false;
        float angle_;
        int pwmIndex_;
        bool inverse_;
        int offset_;
        int range_;
        float adjust_angle_;
    };

}