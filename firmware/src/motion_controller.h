#pragma once

#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "hexapod.h"
#include "movement_profile.h"

namespace motion {

enum class Unit : uint8_t {
    Continuous = 0,
    Cycles,
    Steps,
    Distance,     // 米
    Angle,        // 度
};

struct Action {
    hexapod::MovementMode mode = hexapod::MOVEMENT_STANDBY;
    Unit unit = Unit::Continuous;
    float value = 0.0f;
    float speed = 0.0f;         // 可选速度覆盖
    uint32_t sequenceId = 0;    // 0 表示非序列动作
    bool sequenceTail = false;  // 是否为序列最后一段
};

class MotionController {
public:
    static MotionController& instance();

    void begin();
    void setSequenceCallback(void (*callback)(uint32_t sequenceId));

    bool enqueue(const Action& action);
    bool enqueueSequence(const Action* actions, size_t count);
    void clear(const char* reason = nullptr);

    bool hasActiveAction() const;
    hexapod::MovementMode activeMode() const;

    void onLoopTick(hexapod::MovementMode executedMode, uint32_t elapsedMs);

private:
    struct QueueItem {
        Action action;
    };

    class ActionQueue {
    public:
        bool push(const Action& action);
        bool pop(Action& action);
        void clear();
        bool empty() const { return count_ == 0; }
    private:
        static constexpr size_t kMaxActions = 8;
        Action buffer_[kMaxActions];
        size_t head_ = 0;
        size_t tail_ = 0;
        size_t count_ = 0;
    };

    struct ActiveState {
        Action action;
        float targetCycles = 0.0f;
        float completedCycles = 0.0f;
        bool inUse = false;
        bool restoreSpeed = false;
        float previousSpeed = 0.0f;
    };

private:
    MotionController() = default;
    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;

    void startNextAction();
    void startAction(const Action& action);
    void finishAction();
    float convertToCycles(const Action& action) const;
    float calculateCycleDurationMs(const Action& action) const;
    float sanitizedSpeed(const Action& action) const;

private:
    ActionQueue queue_;
    ActiveState active_;
    void (*sequenceCallback_)(uint32_t) = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
};

MotionController& controller();

}

