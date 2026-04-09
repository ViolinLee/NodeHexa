#include "performance_controller.h"

#include <cstdio>

#include "debug.h"

namespace performance {

using motion::Action;
using motion::Unit;
using namespace hexapod;

namespace {

constexpr size_t kPerformanceSequenceLength = 5;
constexpr float kBeatSwayCyclesPerRound = 2.0f;

Action makeCyclesAction(MovementMode mode, float cycles) {
    Action action;
    action.mode = mode;
    action.unit = Unit::Cycles;
    action.value = cycles;
    return action;
}

MovementMode counterpartOf(MovementMode mode) {
    switch (mode) {
        case MOVEMENT_TURNLEFT:
            return MOVEMENT_TURNRIGHT;
        case MOVEMENT_TURNRIGHT:
            return MOVEMENT_TURNLEFT;
        case MOVEMENT_SHIFTLEFT:
            return MOVEMENT_SHIFTRIGHT;
        case MOVEMENT_SHIFTRIGHT:
            return MOVEMENT_SHIFTLEFT;
        default:
            return MOVEMENT_STANDBY;
    }
}

Action makeFreestylePostureAction(int index) {
    switch (index) {
        case 0:
            return makeCyclesAction(MOVEMENT_ROTATEX, 1.0f);
        case 1:
            return makeCyclesAction(MOVEMENT_ROTATEY, 1.0f);
        case 2:
            return makeCyclesAction(MOVEMENT_ROTATEZ, 1.0f);
        case 3:
        default:
            return makeCyclesAction(MOVEMENT_TWIST, 1.0f);
    }
}

Action makeFreestyleAccentAction(int index) {
    switch (index) {
        case 0:
            return makeCyclesAction(MOVEMENT_TURNLEFT, 1.0f);
        case 1:
            return makeCyclesAction(MOVEMENT_TURNRIGHT, 1.0f);
        case 2:
            return makeCyclesAction(MOVEMENT_SHIFTLEFT, 1.0f);
        case 3:
        default:
            return makeCyclesAction(MOVEMENT_SHIFTRIGHT, 1.0f);
    }
}

Action makeFreestyleTravelAction(int index) {
    return (index == 0)
        ? makeCyclesAction(MOVEMENT_FORWARD, 1.0f)
        : makeCyclesAction(MOVEMENT_BACKWARD, 1.0f);
}

void attachSequenceMetadata(Action* actions, size_t count, uint32_t sequenceId) {
    for (size_t i = 0; i < count; ++i) {
        actions[i].sequenceId = sequenceId;
        actions[i].sequenceTail = (i + 1 == count);
    }
}

bool enqueueActionSequence(Action* actions, size_t count, uint32_t sequenceId, String& error) {
    attachSequenceMetadata(actions, count, sequenceId);
    if (!motion::controller().enqueueSequence(actions, count)) {
        error = "queue full";
        return false;
    }
    return true;
}

}  // namespace

Controller& Controller::instance() {
    static Controller inst;
    return inst;
}

Controller& controller() {
    return Controller::instance();
}

const char* kindToKey(Kind kind) {
    switch (kind) {
        case Kind::Freestyle:
            return "freestyle";
        case Kind::BeatSway:
            return "beatsway";
        case Kind::Showtime:
            return "showtime";
        case Kind::None:
        default:
            return "none";
    }
}

bool isSupported(Kind kind) {
    return kind == Kind::Freestyle
        || kind == Kind::BeatSway
        || kind == Kind::Showtime;
}

void Controller::begin() {
    if (rngState_ == 0) {
        rngState_ = micros();
        if (rngState_ == 0) {
            rngState_ = 0x13579BDFu;
        }
    }
}

bool Controller::start(Kind kind,
                       bool repeat,
                       uint32_t requestedSequenceId,
                       String& error,
                       uint32_t& acceptedSequenceId) {
    begin();

    if (!isSupported(kind)) {
        error = "unsupported performance";
        return false;
    }

    motion::controller().clear("[Performance] start override");
    clear("[Performance] state reset");

    State nextState;
    nextState.kind = kind;
    nextState.repeat = repeat;
    nextState.active = true;
    nextState.sequenceId = requestedSequenceId ? requestedSequenceId : nextSequenceId();

    if (!enqueueRound(nextState, false, error)) {
        clear("[Performance] failed to start");
        return false;
    }

    state_ = nextState;
    acceptedSequenceId = nextState.sequenceId;

    char buffer[120];
    snprintf(buffer,
             sizeof(buffer),
             "[Performance] start kind=%s repeat=%d sequence=%u",
             kindToKey(nextState.kind),
             nextState.repeat ? 1 : 0,
             nextState.sequenceId);
    LOG_INFO(buffer);
    return true;
}

void Controller::clear(const char* reason) {
    if (state_.active && reason) {
        LOG_INFO(reason);
    }
    state_ = State{};
}

void Controller::onSequenceComplete(uint32_t sequenceId) {
    if (!state_.active || state_.sequenceId != sequenceId) {
        return;
    }

    if (!state_.repeat) {
        clear("[Performance] completed");
        return;
    }

    State nextState = state_;
    nextState.sequenceId = nextSequenceId();

    String error;
    if (!enqueueRound(nextState, false, error)) {
        char buffer[128];
        snprintf(buffer,
                 sizeof(buffer),
                 "[Performance] repeat enqueue failed: %s",
                 error.c_str());
        LOG_INFO(buffer);
        clear("[Performance] repeat stopped");
        return;
    }

    state_.sequenceId = nextState.sequenceId;
}

bool Controller::enqueueRound(const State& state, bool clearQueue, String& error) {
    if (clearQueue) {
        motion::controller().clear("[Performance] queue reset");
    }

    switch (state.kind) {
        case Kind::Freestyle:
            return enqueueFreestyle(state.sequenceId, error);
        case Kind::BeatSway:
            return enqueueBeatSway(state.sequenceId, error);
        case Kind::Showtime:
            return enqueueShowtime(state.sequenceId, error);
        case Kind::None:
        default:
            error = "invalid performance kind";
            return false;
    }
}

bool Controller::enqueueFreestyle(uint32_t sequenceId, String& error) {
    Action actions[kPerformanceSequenceLength];

    actions[0] = makeFreestylePostureAction(nextIndex(4));
    actions[1] = makeFreestyleAccentAction(nextIndex(4));

    do {
        actions[2] = makeFreestylePostureAction(nextIndex(4));
    } while (actions[2].mode == actions[0].mode);

#ifdef ROBOT_MODEL_NODEQUADMINI
    const MovementMode paired = counterpartOf(actions[1].mode);
    if (paired != MOVEMENT_STANDBY) {
        actions[3] = makeCyclesAction(paired, 1.0f);
    } else {
        do {
            actions[3] = makeFreestyleAccentAction(nextIndex(4));
        } while (actions[3].mode == actions[1].mode);
    }

    do {
        actions[4] = makeFreestylePostureAction(nextIndex(4));
    } while (actions[4].mode == actions[2].mode);
#else
    const bool useTravel = (nextRandom() & 0x3u) == 0u;
    if (useTravel) {
        actions[3] = makeFreestyleTravelAction(nextIndex(2));
        do {
            actions[4] = makeFreestylePostureAction(nextIndex(4));
        } while (actions[4].mode == actions[2].mode);
    } else {
        do {
            actions[3] = makeFreestyleAccentAction(nextIndex(4));
        } while (actions[3].mode == actions[1].mode);

        const MovementMode paired = counterpartOf(actions[3].mode);
        if (paired != MOVEMENT_STANDBY) {
            actions[4] = makeCyclesAction(paired, 1.0f);
        } else {
            actions[4] = makeFreestylePostureAction(nextIndex(4));
        }
    }
#endif

    return enqueueActionSequence(actions, kPerformanceSequenceLength, sequenceId, error);
}

bool Controller::enqueueShowtime(uint32_t sequenceId, String& error) {
#ifdef ROBOT_MODEL_NODEQUADMINI
    Action actions[kPerformanceSequenceLength] = {
        makeCyclesAction(MOVEMENT_ROTATEZ, 1.0f),
        makeCyclesAction(MOVEMENT_TWIST, 1.0f),
        makeCyclesAction(MOVEMENT_TURNLEFT, 1.0f),
        makeCyclesAction(MOVEMENT_TURNRIGHT, 1.0f),
        makeCyclesAction(MOVEMENT_ROTATEY, 1.0f),
    };
#else
    Action actions[kPerformanceSequenceLength] = {
        makeCyclesAction(MOVEMENT_ROTATEZ, 1.0f),
        makeCyclesAction(MOVEMENT_TWIST, 1.0f),
        makeCyclesAction(MOVEMENT_SHIFTLEFT, 1.0f),
        makeCyclesAction(MOVEMENT_SHIFTRIGHT, 1.0f),
        makeCyclesAction(MOVEMENT_ROTATEY, 1.0f),
    };
#endif

    return enqueueActionSequence(actions, kPerformanceSequenceLength, sequenceId, error);
}

bool Controller::enqueueBeatSway(uint32_t sequenceId, String& error) {
#ifdef ROBOT_MODEL_NODEQUADMINI
    Action actions[kPerformanceSequenceLength] = {
        makeCyclesAction(MOVEMENT_ROTATEX, 1.0f),
        makeCyclesAction(MOVEMENT_ROTATEZ, 1.0f),
        makeCyclesAction(MOVEMENT_TWIST, 1.0f),
        makeCyclesAction(MOVEMENT_ROTATEZ, 1.0f),
        makeCyclesAction(MOVEMENT_ROTATEY, 1.0f),
    };
    return enqueueActionSequence(actions, kPerformanceSequenceLength, sequenceId, error);
#else
    Action action = makeCyclesAction(MOVEMENT_BEATSWAY, kBeatSwayCyclesPerRound);
    action.sequenceId = sequenceId;
    action.sequenceTail = true;

    if (!motion::controller().enqueue(action)) {
        error = "queue full";
        return false;
    }
    return true;
#endif
}

uint32_t Controller::nextSequenceId() {
    uint32_t next = millis();
    if (next == 0) {
        next = 1;
    }
    if (next <= sequenceCounter_) {
        next = sequenceCounter_ + 1;
    }
    sequenceCounter_ = next;
    return next;
}

uint32_t Controller::nextRandom() {
    if (rngState_ == 0) {
        begin();
    }
    rngState_ ^= (rngState_ << 13);
    rngState_ ^= (rngState_ >> 17);
    rngState_ ^= (rngState_ << 5);
    return rngState_;
}

int Controller::nextIndex(int limit) {
    if (limit <= 1) {
        return 0;
    }
    return static_cast<int>(nextRandom() % static_cast<uint32_t>(limit));
}

}  // namespace performance
