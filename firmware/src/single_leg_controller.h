#pragma once

#include <Arduino.h>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "base.h"

namespace singleleg {

struct InputAxes {
    float lx = 0.0f;
    float ly = 0.0f;
    float rz = 0.0f;
};

class Controller {
public:
    static Controller& instance();

    void begin();

    bool start(uint8_t legIndex, String& error);
    void updateInput(const InputAxes& axes);
    void stop(const char* reason = nullptr);

    bool isActive() const;
    uint8_t activeLegIndex() const;

    void onLoopTick(uint32_t elapsedMs);

private:
    struct State {
        bool active = false;
        uint8_t legIndex = 0;
        hexapod::Point3D standbyTipLocal;
        hexapod::Point3D currentTipWorld;
        InputAxes axes;
        float maxLocalRadius = 0.0f;
        uint32_t lastInputMs = 0;
    };

private:
    Controller() = default;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void clampTargetLocal(const State& state, hexapod::Point3D& localTip) const;

private:
    State state_;
    mutable SemaphoreHandle_t mutex_ = nullptr;
};

Controller& controller();

}  // namespace singleleg
