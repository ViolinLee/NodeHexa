#include "movement.h"
#include "debug.h"
#include "config.h"

#include <cstdlib>

namespace hexapod {

    extern const MovementTable& standbyTable();
    extern const MovementTable& forwardTable();
    extern const MovementTable& forwardfastTable();
    extern const MovementTable& backwardTable();
    extern const MovementTable& turnleftTable();
    extern const MovementTable& turnrightTable();
    extern const MovementTable& shiftleftTable();
    extern const MovementTable& shiftrightTable();
    extern const MovementTable& climbTable();
    extern const MovementTable& rotatexTable();
    extern const MovementTable& rotateyTable();
    extern const MovementTable& rotatezTable();
    extern const MovementTable& twistTable();

    const MovementTable kTable[MOVEMENT_TOTAL] {
        standbyTable(),
        forwardTable(),
        forwardfastTable(),
        backwardTable(),
        turnleftTable(),
        turnrightTable(),
        shiftleftTable(),
        shiftrightTable(),
        climbTable(),
        rotatexTable(),
        rotateyTable(),
        rotatezTable(),
        twistTable(),
    };

    Movement::Movement(MovementMode mode):
        mode_{mode}, transiting_{false}, speed_{config::defaultSpeed}
    {
    }

    void Movement::setMode(MovementMode newMode) {

        if (!kTable[newMode].entries) {
            LOG_INFO("Error: null movement of mode(%d)!", newMode);
            return;
        }

        mode_ = newMode;

        const MovementTable& table = kTable[mode_];

        index_ = table.entries[std::rand() % table.entriesCount];
        int actualDuration = (int)(table.stepDuration / speed_);
        int actualSwitchDuration = (int)(config::movementSwitchDuration / speed_);
        remainTime_ = actualSwitchDuration > actualDuration ? actualSwitchDuration : actualDuration;
    }

    const Locations& Movement::next(int elapsed) {

        const MovementTable& table = kTable[mode_];

        // Calculate actual step duration based on speed
        int actualStepDuration = (int)(table.stepDuration / speed_);

        if (elapsed <= 0)
            elapsed = actualStepDuration;

        if (remainTime_ <= 0) {
            index_ = (index_ + 1)%table.length;
            remainTime_ = actualStepDuration;
        }
        if (elapsed >= remainTime_)
            elapsed = remainTime_;

        auto ratio = (float)elapsed / remainTime_;
        position_ += (table.table[index_] - position_)*ratio;
        remainTime_ -= elapsed;

        return position_;
    }

    void Movement::setSpeed(float speed) {
        // Clamp speed to valid range
        if (speed < config::minSpeed)
            speed = config::minSpeed;
        else if (speed > config::maxSpeed)
            speed = config::maxSpeed;
        
        speed_ = speed;
    }

    float Movement::getSpeed() const {
        return speed_;
    }
}