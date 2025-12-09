#pragma once

#include <cstdint>

namespace quad {

    // 四足版本的舵机类：使用单片 PCA9685，顺序映射 12 路通道
    class ServoQuad {
    public:
        static void init(void);

    public:
        ServoQuad(int legIndex, int partIndex);

        // angle: 0 means center, range is about -60~60
        void setAngle(float angle);
        float getAngle(void);

        void getParameter(int& offset) {
            offset = offset_;
        }

        void setParameter(int offset, bool update = true) {
            offset_ = offset;
            if (update)
                setAngle(angle_);
        }

    private:
        float angle_{0.0f};
        int   pwmIndex_{0};
        bool  inverse_{false};
        int   offset_{0};
        int   range_{60};
        float adjust_angle_{0.0f};
    };

} // namespace quad

