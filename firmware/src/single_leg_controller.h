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

struct Status {
    bool active, returning, faulted, limited;
    uint8_t legIndex;
    hexapod::Point3D tip;
};

class Controller {
public:
    static Controller& instance();

    void begin();

    bool start(uint8_t legIndex, String& error);
    void updateInput(const InputAxes& axes);
    void stop(const char* reason = nullptr, bool restoreStandby = true);

    bool isActive() const;
    uint8_t activeLegIndex() const;
    Status status() const;

    void onLoopTick(uint32_t elapsedMs);

private:
    struct State {
        bool active = false;
        uint8_t legIndex = 0;
        hexapod::Point3D standbyTipWorld{0, 0, 0};
        hexapod::Point3D currentTipWorld{0, 0, 0};
        InputAxes axes;
        bool returning = false;
        bool faulted = false;
        bool limited = false;
        int pendingLeg = -1;
        uint32_t lastInputMs = 0;
    };

private:
    Controller() = default;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;



private:
    State state_;
    mutable SemaphoreHandle_t mutex_ = nullptr;
};

Controller& controller();

}  // namespace singleleg
