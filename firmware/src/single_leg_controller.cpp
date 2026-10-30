#include "single_leg_controller.h"

#include <cmath>

#include "debug.h"
#include "robot.h"

namespace singleleg {

namespace {

constexpr float kSpeedXmmPerSec = 45.0f;
constexpr float kSpeedYmmPerSec = 45.0f;
constexpr float kSpeedZmmPerSec = 35.0f;

constexpr uint32_t kInputTimeoutMs = 180;

float clampf(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

}  // namespace

Controller& Controller::instance() {
    static Controller inst;
    return inst;
}

Controller& controller() {
    return Controller::instance();
}

void Controller::begin() {
    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
    }
}

bool Controller::start(uint8_t legIndex, String& error) {
    begin();
    if (!hexapod::Robot || !hexapod::Robot->supportsSingleLegControl() || legIndex >= 6) {
        error = "invalid or unsupported single leg";
        return false;
    }
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        error = "single leg controller busy";
        return false;
    }
    hexapod::Point3D standby;
    if (!hexapod::Robot->beginSingleLegControl(legIndex, standby)) {
        error = "standby pose exceeds calibrated limits";
        xSemaphoreGive(mutex_);
        return false;
    }
    state_.active = true;
    state_.returning = true;
    state_.faulted = false;
    state_.pendingLeg = legIndex;
    state_.standbyTipWorld = standby;
    state_.axes = InputAxes{};
    state_.lastInputMs = millis();
    xSemaphoreGive(mutex_);
    return true;
}

void Controller::updateInput(const InputAxes& axes) {
    begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return;
    if (state_.active && !state_.faulted) {
        if (std::isfinite(axes.lx) && std::isfinite(axes.ly) && std::isfinite(axes.rz)) {
            state_.axes.lx = clampf(axes.lx, -1.0f, 1.0f);
            state_.axes.ly = clampf(axes.ly, -1.0f, 1.0f);
            state_.axes.rz = clampf(axes.rz, -1.0f, 1.0f);
        } else {
            state_.axes = InputAxes{};
        }
        state_.lastInputMs = millis();
    }
    xSemaphoreGive(mutex_);
}

void Controller::stop(const char* reason, bool restoreStandby) {
    begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return;
    const bool wasActive = state_.active;
    if (wasActive && restoreStandby) {
        state_.faulted = false;
        state_.axes = InputAxes{};
        state_.returning = true;
        state_.pendingLeg = -1;
    } else {
        // Power protection must never initiate a return motion.
        state_ = State{};
    }
    xSemaphoreGive(mutex_);
    if (wasActive && reason) LOG_INFO(reason);
}

bool Controller::isActive() const {
    return status().active;
}

uint8_t Controller::activeLegIndex() const {
    return status().legIndex;
}

Status Controller::status() const {
    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return Status{};
    const Status result{state_.active, state_.returning, state_.faulted, state_.limited,
        state_.pendingLeg >= 0 ? static_cast<uint8_t>(state_.pendingLeg) : state_.legIndex,
        state_.currentTipWorld};
    if (mutex_) xSemaphoreGive(mutex_);
    return result;
}

void Controller::onLoopTick(uint32_t elapsedMs) {
    begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) return;
    // Keep state and output in one critical section: a stopped/restarted session
    // cannot receive a stale tick after the WebSocket callback changes it.
    if (!state_.active || state_.faulted || !hexapod::Robot) {
        xSemaphoreGive(mutex_);
        return;
    }
    if (millis() - state_.lastInputMs > kInputTimeoutMs) state_.axes = InputAxes{};
    if (state_.returning) {
        bool complete = false;
        if (!hexapod::Robot->returnSingleLegControl(elapsedMs, complete)) {
            state_.faulted = true;
            LOG_INFO("[SingleLeg] return failed; holding last command");
        } else if (complete) {
            hexapod::Robot->endSingleLegControl();
            if (state_.pendingLeg < 0) {
                state_ = State{};
            } else {
                state_.legIndex = static_cast<uint8_t>(state_.pendingLeg);
                state_.pendingLeg = -1;
                state_.returning = false;
                state_.currentTipWorld = state_.standbyTipWorld;
                state_.limited = false;
            }
        }
        xSemaphoreGive(mutex_);
        return;
    }

    const float dt = (elapsedMs > 50 ? 50 : elapsedMs) / 1000.0f;
    const auto axes = state_.axes;
    if (dt <= 0 || (axes.lx == 0 && axes.ly == 0 && axes.rz == 0)) {
        xSemaphoreGive(mutex_);
        return; // No transform roundtrip and no output while the sticks are idle.
    }
    const auto start = state_.currentTipWorld;
    hexapod::Point3D delta(axes.lx * kSpeedXmmPerSec * dt,
                          axes.ly * kSpeedYmmPerSec * dt,
                          axes.rz * kSpeedZmmPerSec * dt);
    auto target = start;
    // First follow the requested vector. If a boundary blocks it, allow the
    // remaining coordinates to slide along the workspace rather than sticking.
    auto advance = [&](const hexapod::Point3D& step) {
        auto candidate = target;
        candidate += step;
        candidate.z_ = std::fmax(candidate.z_, state_.standbyTipWorld.z_);
        const auto movement = candidate - target;
        float lo = 0, hi = 1;
        if (hexapod::Robot->validateSingleLegControl(state_.legIndex, candidate, 60.0f * dt)) {
            target = candidate;
            return;
        }
        for (int i = 0; i < 12; ++i) {
            const float fraction = (lo + hi) * 0.5f;
            auto probe = target;
            probe += movement * fraction;
            if (hexapod::Robot->validateSingleLegControl(state_.legIndex, probe, 60.0f * dt)) lo = fraction;
            else hi = fraction;
        }
        target += movement * lo;
    };
    advance(delta);
    auto remaining = start;
    remaining += delta;
    remaining = remaining - target;
    advance({remaining.x_, 0, 0});
    advance({0, remaining.y_, 0});
    advance({0, 0, remaining.z_});
    const bool limited = std::fabs(target.x_ - start.x_ - delta.x_) > 0.01f
        || std::fabs(target.y_ - start.y_ - delta.y_) > 0.01f
        || std::fabs(target.z_ - start.z_ - delta.z_) > 0.01f;
    if (limited && !state_.limited) LOG_INFO("[SingleLeg] workspace or joint speed boundary");
    state_.limited = limited;
    if (!(target == start)) {
        if (hexapod::Robot->applySingleLegControl(state_.legIndex, target)) {
            state_.currentTipWorld = target;
        } else {
            state_.faulted = true;
            state_.axes = InputAxes{};
            LOG_INFO("[SingleLeg] output failed; holding last command");
        }
    }
    xSemaphoreGive(mutex_);
}

} // namespace singleleg
