#ifdef ROBOT_MODEL_NODEQUADMINI

#include "quad_movement.h"
#include "config.h"
#include "debug.h"

using namespace hexapod;

namespace quadruped {

    // 四种步态对应的多模式离线表
    extern const QuadMovementTable& quad_trot_forwardTable();
    extern const QuadMovementTable& quad_trot_forwardfastTable();
    extern const QuadMovementTable& quad_trot_backwardTable();
    extern const QuadMovementTable& quad_trot_shiftleftTable();
    extern const QuadMovementTable& quad_trot_shiftrightTable();
    extern const QuadMovementTable& quad_trot_turnleftTable();
    extern const QuadMovementTable& quad_trot_turnrightTable();

    extern const QuadMovementTable& quad_walk_forwardTable();
    extern const QuadMovementTable& quad_walk_forwardfastTable();
    extern const QuadMovementTable& quad_walk_backwardTable();
    extern const QuadMovementTable& quad_walk_shiftleftTable();
    extern const QuadMovementTable& quad_walk_shiftrightTable();
    extern const QuadMovementTable& quad_walk_turnleftTable();
    extern const QuadMovementTable& quad_walk_turnrightTable();

    extern const QuadMovementTable& quad_gallop_forwardTable();
    extern const QuadMovementTable& quad_gallop_forwardfastTable();
    extern const QuadMovementTable& quad_gallop_backwardTable();
    extern const QuadMovementTable& quad_gallop_shiftleftTable();
    extern const QuadMovementTable& quad_gallop_shiftrightTable();
    extern const QuadMovementTable& quad_gallop_turnleftTable();
    extern const QuadMovementTable& quad_gallop_turnrightTable();

    extern const QuadMovementTable& quad_creep_forwardTable();
    extern const QuadMovementTable& quad_creep_forwardfastTable();
    extern const QuadMovementTable& quad_creep_backwardTable();
    extern const QuadMovementTable& quad_creep_shiftleftTable();
    extern const QuadMovementTable& quad_creep_shiftrightTable();
    extern const QuadMovementTable& quad_creep_turnleftTable();
    extern const QuadMovementTable& quad_creep_turnrightTable();

    // 姿态动作：不分步态
    extern const QuadMovementTable& quad_rotatexTable();
    extern const QuadMovementTable& quad_rotateyTable();
    extern const QuadMovementTable& quad_rotatezTable();
    extern const QuadMovementTable& quad_twistTable();

    namespace {

        const QuadMovementTable& selectByMode(
            MovementMode mode,
            const QuadMovementTable& standby,
            const QuadMovementTable& forward,
            const QuadMovementTable& forwardfast,
            const QuadMovementTable& backward,
            const QuadMovementTable& shiftleft,
            const QuadMovementTable& shiftright,
            const QuadMovementTable& turnleft,
            const QuadMovementTable& turnright) {
            switch (mode) {
            case MOVEMENT_STANDBY:
                return standby;
            case MOVEMENT_FORWARD:
                return forward;
            case MOVEMENT_FORWARDFAST:
                return forwardfast;
            case MOVEMENT_BACKWARD:
                return backward;
            case MOVEMENT_SHIFTLEFT:
                return shiftleft;
            case MOVEMENT_SHIFTRIGHT:
                return shiftright;
            case MOVEMENT_TURNLEFT:
                return turnleft;
            case MOVEMENT_TURNRIGHT:
                return turnright;
            case MOVEMENT_CLIMB:
                // 四足不实现 climb：重心不稳定，直接退化为待机更安全
                return standby;
            default:
                // 其他暂不支持的模式，退化为待机
                return standby;
            }
        }

        const QuadMovementTable& selectTable(QuadGaitMode gait, MovementMode mode) {
            // 姿态动作：不分步态
            switch (mode) {
            case MOVEMENT_ROTATEX:
                return quad_rotatexTable();
            case MOVEMENT_ROTATEY:
                return quad_rotateyTable();
            case MOVEMENT_ROTATEZ:
                return quad_rotatezTable();
            case MOVEMENT_TWIST:
                return quad_twistTable();
            default:
                break;
            }

            switch (gait) {
            case QUAD_GAIT_TROT:
                return selectByMode(
                    mode,
                    standbyTable(),
                    quad_trot_forwardTable(),
                    quad_trot_forwardfastTable(),
                    quad_trot_backwardTable(),
                    quad_trot_shiftleftTable(),
                    quad_trot_shiftrightTable(),
                    quad_trot_turnleftTable(),
                    quad_trot_turnrightTable());
            case QUAD_GAIT_WALK:
                return selectByMode(
                    mode,
                    standbyTable(),
                    quad_walk_forwardTable(),
                    quad_walk_forwardfastTable(),
                    quad_walk_backwardTable(),
                    quad_walk_shiftleftTable(),
                    quad_walk_shiftrightTable(),
                    quad_walk_turnleftTable(),
                    quad_walk_turnrightTable());
            case QUAD_GAIT_GALLOP:
                return selectByMode(
                    mode,
                    standbyTable(),
                    quad_gallop_forwardTable(),
                    quad_gallop_forwardfastTable(),
                    quad_gallop_backwardTable(),
                    quad_gallop_shiftleftTable(),
                    quad_gallop_shiftrightTable(),
                    quad_gallop_turnleftTable(),
                    quad_gallop_turnrightTable());
            case QUAD_GAIT_CREEP:
                return selectByMode(
                    mode,
                    standbyTable(),
                    quad_creep_forwardTable(),
                    quad_creep_forwardfastTable(),
                    quad_creep_backwardTable(),
                    quad_creep_shiftleftTable(),
                    quad_creep_shiftrightTable(),
                    quad_creep_turnleftTable(),
                    quad_creep_turnrightTable());
            default:
                // 容错：未知 gait 时退化为 trotting
                return selectByMode(
                    mode,
                    standbyTable(),
                    quad_trot_forwardTable(),
                    quad_trot_forwardfastTable(),
                    quad_trot_backwardTable(),
                    quad_trot_shiftleftTable(),
                    quad_trot_shiftrightTable(),
                    quad_trot_turnleftTable(),
                    quad_trot_turnrightTable());
            }
        }
    }

    QuadMovement::QuadMovement(MovementMode mode, QuadGaitMode gait)
        : mode_{mode},
          gaitMode_{gait},
          position_{},
          index_{0},
          remainTime_{0},
          speed_{config::defaultSpeed} {
    }

    void QuadMovement::setMode(MovementMode newMode) {
        mode_ = newMode;

        const QuadMovementTable& table = currentTable();
        if (!table.entries || table.length <= 0) {
            LOG_INFO("[QuadMovement] Error: null movement of mode(%d)!", newMode);
            return;
        }

        index_ = table.entries[0];
        int actualDuration = static_cast<int>(table.stepDuration / speed_);
        int actualSwitchDuration = static_cast<int>(config::movementSwitchDuration / speed_);
        remainTime_ = actualSwitchDuration > actualDuration ? actualSwitchDuration : actualDuration;

        // 初始化当前位置为当前关键帧，避免首次插值从全零跳变
        for (int i = 0; i < 4; ++i) {
            position_.p[i] = table.table[index_].p[i];
        }
    }

    void QuadMovement::setGaitMode(QuadGaitMode gait) {
        gaitMode_ = gait;
        // 保持当前 MovementMode，重新选择表
        setMode(mode_);
    }

    const QuadLocations& QuadMovement::next(int elapsedMs) {
        const QuadMovementTable& table = currentTable();

        int actualStepDuration = static_cast<int>(table.stepDuration / speed_);
        if (elapsedMs <= 0)
            elapsedMs = actualStepDuration;

        if (remainTime_ <= 0) {
            index_ = (index_ + 1) % table.length;
            remainTime_ = actualStepDuration;
        }
        if (elapsedMs >= remainTime_)
            elapsedMs = remainTime_;

        float ratio = static_cast<float>(elapsedMs) / static_cast<float>(remainTime_);
        for (int i = 0; i < 4; ++i) {
            hexapod::Point3D diff = table.table[index_].p[i] - position_.p[i];
            position_.p[i] += diff * ratio;
        }

        remainTime_ -= elapsedMs;
        return position_;
    }

    void QuadMovement::setSpeed(float speed) {
        if (speed < config::minSpeed)
            speed = config::minSpeed;
        else if (speed > config::maxSpeed)
            speed = config::maxSpeed;
        speed_ = speed;
    }

    float QuadMovement::getSpeed() const {
        return speed_;
    }

    const QuadMovementTable& QuadMovement::currentTable() const {
        return selectTable(gaitMode_, mode_);
    }

} // namespace quadruped

#endif // ROBOT_MODEL_NODEQUADMINI
