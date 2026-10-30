#pragma once

#include "servo_mapping.h"

#include <cstdint>

namespace quadruped {

    // 四足版本的舵机类：使用单片 PCA9685，顺序映射 12 路通道
    class ServoQuad {
    public:
        static void init(void);

    public:
        ServoQuad(int legIndex, int partIndex);

        // angle: 0 means center, range is about -60~60
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
            return {inverse_, adjust_angle_, static_cast<float>(range_), offset_, true};
        }
        bool commanded_ = false;
        float angle_{0.0f};
        int   pwmIndex_{0};
        bool  inverse_{false};
        int   offset_{0};
        int   range_{60};
        float adjust_angle_{0.0f};
    };

} // namespace quadruped

