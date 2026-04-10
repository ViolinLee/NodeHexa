#include "single_leg_controller.h"

#include <cmath>

#include "debug.h"
#include "robot.h"

namespace singleleg {

namespace {

constexpr float kMaxLocalDeltaX = 18.0f;
constexpr float kMaxLocalDeltaY = 18.0f;
constexpr float kMaxLocalLiftZ = 22.0f;
constexpr float kMaxLocalRadiusDelta = 10.0f;

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

    if (!hexapod::Robot || !hexapod::Robot->supportsSingleLegControl()) {
        error = "single leg control is not supported";
        return false;
    }

    if (legIndex >= 6) {
        error = "invalid leg index";
        return false;
    }

    hexapod::Point3D standbyTipWorld;
    if (!hexapod::Robot->beginSingleLegControl(static_cast<int>(legIndex), standbyTipWorld)) {
        error = "failed to enter single leg control";
        return false;
    }

    hexapod::Point3D standbyTipLocal;
    if (!hexapod::Robot->singleLegWorldToLocal(static_cast<int>(legIndex), standbyTipWorld, standbyTipLocal)) {
        hexapod::Robot->endSingleLegControl();
        error = "failed to read local standby pose";
        return false;
    }

    if (!hexapod::Robot->applySingleLegControl(static_cast<int>(legIndex), standbyTipWorld)) {
        hexapod::Robot->endSingleLegControl();
        error = "failed to apply single leg control";
        return false;
    }

    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        hexapod::Robot->endSingleLegControl();
        error = "single leg controller busy";
        return false;
    }

    state_ = State{};
    state_.active = true;
    state_.legIndex = legIndex;
    state_.standbyTipLocal = standbyTipLocal;
    state_.currentTipWorld = standbyTipWorld;
    state_.maxLocalRadius = std::sqrt(standbyTipLocal.x_ * standbyTipLocal.x_ + standbyTipLocal.y_ * standbyTipLocal.y_)
        + kMaxLocalRadiusDelta;
    state_.lastInputMs = millis();

    xSemaphoreGive(mutex_);
    return true;
}

void Controller::updateInput(const InputAxes& axes) {
    begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (state_.active) {
        state_.axes = axes;
        state_.lastInputMs = millis();
    }
    xSemaphoreGive(mutex_);
}

void Controller::stop(const char* reason) {
    begin();
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    const bool wasActive = state_.active;
    state_ = State{};
    xSemaphoreGive(mutex_);

    if (wasActive && hexapod::Robot) {
        hexapod::Robot->endSingleLegControl();
    }

    if (wasActive && reason) {
        LOG_INFO(reason);
    }
}

bool Controller::isActive() const {
    if (!mutex_) {
        return state_.active;
    }
    bool active = state_.active;
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        active = state_.active;
        xSemaphoreGive(mutex_);
    }
    return active;
}

uint8_t Controller::activeLegIndex() const {
    if (!mutex_) {
        return state_.legIndex;
    }
    uint8_t legIndex = state_.legIndex;
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        legIndex = state_.legIndex;
        xSemaphoreGive(mutex_);
    }
    return legIndex;
}

void Controller::onLoopTick(uint32_t elapsedMs) {
    begin();

    State snapshot;
    bool shouldApply = false;

    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }

    if (state_.active) {
        const uint32_t now = millis();
        if (now - state_.lastInputMs > kInputTimeoutMs) {
            state_.axes = InputAxes{};
        }

        snapshot = state_;
        shouldApply = true;
    }

    xSemaphoreGive(mutex_);

    if (!shouldApply) {
        return;
    }

    hexapod::Point3D rawWorldTip = snapshot.currentTipWorld;
    rawWorldTip.x_ += snapshot.axes.lx * kSpeedXmmPerSec * (static_cast<float>(elapsedMs) / 1000.0f);
    rawWorldTip.y_ += snapshot.axes.ly * kSpeedYmmPerSec * (static_cast<float>(elapsedMs) / 1000.0f);
    rawWorldTip.z_ += snapshot.axes.rz * kSpeedZmmPerSec * (static_cast<float>(elapsedMs) / 1000.0f);

    hexapod::Point3D localTip;
    if (!hexapod::Robot || !hexapod::Robot->singleLegWorldToLocal(static_cast<int>(snapshot.legIndex), rawWorldTip, localTip)) {
        stop("[SingleLeg] world->local failed");
        return;
    }

    clampTargetLocal(snapshot, localTip);

    hexapod::Point3D safeWorldTip;
    if (!hexapod::Robot->singleLegLocalToWorld(static_cast<int>(snapshot.legIndex), localTip, safeWorldTip)) {
        stop("[SingleLeg] local->world failed");
        return;
    }

    if (!hexapod::Robot->applySingleLegControl(static_cast<int>(snapshot.legIndex), safeWorldTip)) {
        stop("[SingleLeg] apply failed");
        return;
    }

    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
        if (state_.active && state_.legIndex == snapshot.legIndex) {
            state_.currentTipWorld = safeWorldTip;
        }
        xSemaphoreGive(mutex_);
    }
}

void Controller::clampTargetLocal(const State& state, hexapod::Point3D& localTip) const {
    localTip.x_ = clampf(localTip.x_,
                         state.standbyTipLocal.x_ - kMaxLocalDeltaX,
                         state.standbyTipLocal.x_ + kMaxLocalDeltaX);
    localTip.y_ = clampf(localTip.y_,
                         state.standbyTipLocal.y_ - kMaxLocalDeltaY,
                         state.standbyTipLocal.y_ + kMaxLocalDeltaY);
    localTip.z_ = clampf(localTip.z_,
                         state.standbyTipLocal.z_,
                         state.standbyTipLocal.z_ + kMaxLocalLiftZ);

    const float radius = std::sqrt(localTip.x_ * localTip.x_ + localTip.y_ * localTip.y_);
    if (radius > state.maxLocalRadius && radius > 1e-4f) {
        const float scale = state.maxLocalRadius / radius;
        localTip.x_ *= scale;
        localTip.y_ *= scale;
    }
}

}  // namespace singleleg
