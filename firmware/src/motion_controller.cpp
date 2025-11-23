#include "motion_controller.h"

#include <cmath>

#include "debug.h"
#include "movement.h"

namespace motion {

using namespace hexapod;

MotionController& MotionController::instance() {
    static MotionController inst;
    return inst;
}

MotionController& controller() {
    return MotionController::instance();
}

void MotionController::begin() {
    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
    }
}

void MotionController::setSequenceCallback(void (*callback)(uint32_t sequenceId)) {
    sequenceCallback_ = callback;
}

bool MotionController::enqueue(const Action& action) {
    if (!mutex_) begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    bool pushed = queue_.push(action);
    if (pushed && !active_.inUse) {
        startNextAction();
    }
    xSemaphoreGive(mutex_);
    return pushed;
}

bool MotionController::enqueueSequence(const Action* actions, size_t count) {
    if (!actions || count == 0) {
        return false;
    }
    bool result = true;
    for (size_t i = 0; i < count; ++i) {
        if (!enqueue(actions[i])) {
            result = false;
            break;
        }
    }
    return result;
}

void MotionController::clear(const char* reason) {
    if (!mutex_) begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    queue_.clear();
    if (active_.inUse && active_.restoreSpeed) {
        Hexapod.setMovementSpeed(active_.previousSpeed);
    }
    active_ = ActiveState{};
    if (reason) {
        LOG_INFO(reason);
    }
    xSemaphoreGive(mutex_);
}

bool MotionController::hasActiveAction() const {
    return active_.inUse;
}

hexapod::MovementMode MotionController::activeMode() const {
    return active_.inUse ? active_.action.mode : hexapod::MOVEMENT_STANDBY;
}

void MotionController::onLoopTick(MovementMode executedMode, uint32_t elapsedMs) {
    if (!active_.inUse || active_.action.mode != executedMode) {
        return;
    }

    if (active_.targetCycles > 0.0f) {
        float duration = calculateCycleDurationMs(active_.action);
        if (duration <= 1e-3f) {
            return;
        }
        active_.completedCycles += static_cast<float>(elapsedMs) / duration;
        if (active_.completedCycles >= active_.targetCycles - 1e-3f) {
            finishAction();
        }
        return;
    }

    if (active_.targetDurationMs > 0) {
        active_.elapsedDurationMs += elapsedMs;
        if (active_.elapsedDurationMs >= active_.targetDurationMs) {
            finishAction();
        }
    }
}

void MotionController::startNextAction() {
    Action action;
    if (!queue_.pop(action)) {
        return;
    }
    startAction(action);
}

void MotionController::startAction(const Action& action) {
    active_.action = action;
    active_.completedCycles = 0.0f;
    active_.elapsedDurationMs = 0;
    active_.targetCycles = 0.0f;
    active_.targetDurationMs = 0;
    active_.inUse = true;

    if (action.unit == Unit::DurationMs) {
        active_.targetDurationMs = (action.durationMs > 0)
            ? action.durationMs
            : static_cast<uint32_t>(action.value);
    } else if (action.unit != Unit::Continuous) {
        active_.targetCycles = convertToCycles(action);
    }

    active_.restoreSpeed = false;
    if (action.speed >= hexapod::config::minSpeed && action.speed <= hexapod::config::maxSpeed) {
        active_.previousSpeed = Hexapod.getMovementSpeed();
        Hexapod.setMovementSpeed(action.speed);
        active_.restoreSpeed = true;
    }

    char buffer[120];
    snprintf(buffer, sizeof(buffer), "[MotionController] Start action mode=%d unit=%d targetCycles=%.2f targetDuration=%u sequence=%u",
             action.mode, static_cast<int>(action.unit), active_.targetCycles, active_.targetDurationMs, action.sequenceId);
    LOG_INFO(buffer);
}

void MotionController::finishAction() {
    if (!active_.inUse) {
        return;
    }

    if (active_.restoreSpeed) {
        Hexapod.setMovementSpeed(active_.previousSpeed);
    }

    uint32_t sequenceId = active_.action.sequenceId;
    bool isTail = active_.action.sequenceTail;
    active_ = ActiveState{};

    if (sequenceId != 0 && isTail && sequenceCallback_) {
        sequenceCallback_(sequenceId);
    }

    if (!queue_.empty()) {
        startNextAction();
    }
}

float MotionController::convertToCycles(const Action& action) const {
    const auto& metrics = getMovementMetrics(action.mode);
    float value = std::fabs(action.value);
    switch (action.unit) {
        case Unit::Cycles:
            return value;
        case Unit::Steps:
            return (metrics.stepsPerCycle > 0.0f) ? value / metrics.stepsPerCycle : 0.0f;
        case Unit::Distance:
            return (metrics.distancePerCycleMeters > 0.0f) ? value / metrics.distancePerCycleMeters : 0.0f;
        case Unit::Angle:
            return (metrics.degreesPerCycle > 0.0f) ? value / metrics.degreesPerCycle : 0.0f;
        default:
            break;
    }
    return 0.0f;
}

float MotionController::calculateCycleDurationMs(const Action& action) const {
    float speed = Hexapod.getMovementSpeed();
    if (speed < hexapod::config::minSpeed) {
        speed = hexapod::config::minSpeed;
    }
    const auto& table = getMovementTable(action.mode);
    return (static_cast<float>(table.length) * (static_cast<float>(table.stepDuration) / speed));
}

bool MotionController::ActionQueue::push(const Action& action) {
    if (count_ >= kMaxActions) {
        return false;
    }
    buffer_[tail_] = action;
    tail_ = (tail_ + 1) % kMaxActions;
    ++count_;
    return true;
}

bool MotionController::ActionQueue::pop(Action& action) {
    if (count_ == 0) {
        return false;
    }
    action = buffer_[head_];
    head_ = (head_ + 1) % kMaxActions;
    --count_;
    return true;
}

void MotionController::ActionQueue::clear() {
    head_ = tail_ = count_ = 0;
}

} // namespace motion

