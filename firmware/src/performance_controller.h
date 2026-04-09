#pragma once

#include <cstdint>

#include <Arduino.h>

#include "motion_controller.h"

namespace performance {

enum class Kind : uint8_t {
    None = 0,
    Freestyle,
    BeatSway,
    Showtime,
};

class Controller {
public:
    static Controller& instance();

    void begin();

    bool start(Kind kind, bool repeat, uint32_t requestedSequenceId, String& error, uint32_t& acceptedSequenceId);
    void clear(const char* reason = nullptr);
    void onSequenceComplete(uint32_t sequenceId);

    bool isActive() const { return state_.active; }
    Kind activeKind() const { return state_.active ? state_.kind : Kind::None; }

private:
    struct State {
        Kind kind = Kind::None;
        bool repeat = false;
        bool active = false;
        uint32_t sequenceId = 0;
    };

private:
    Controller() = default;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    bool enqueueRound(const State& state, bool clearQueue, String& error);
    bool enqueueFreestyle(uint32_t sequenceId, String& error);
    bool enqueueShowtime(uint32_t sequenceId, String& error);
    bool enqueueBeatSway(uint32_t sequenceId, String& error);

    uint32_t nextSequenceId();
    uint32_t nextRandom();
    int nextIndex(int limit);

private:
    State state_;
    uint32_t rngState_ = 0;
    uint32_t sequenceCounter_ = 0;
};

Controller& controller();

const char* kindToKey(Kind kind);
bool isSupported(Kind kind);

}  // namespace performance
