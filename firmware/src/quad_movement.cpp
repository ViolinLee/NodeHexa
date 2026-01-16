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

        inline bool isPostureMode(MovementMode mode) {
            return mode == MOVEMENT_ROTATEX
                || mode == MOVEMENT_ROTATEY
                || mode == MOVEMENT_ROTATEZ
                || mode == MOVEMENT_TWIST;
        }

        inline int switchGroupOf(MovementMode mode) {
            // 组内切换：要求两表存在等效 entry（同一姿态帧），在 entry 处可无缝跳表
            switch (mode) {
            case MOVEMENT_FORWARD:
            case MOVEMENT_BACKWARD:
                return 1;
            case MOVEMENT_TURNLEFT:
            case MOVEMENT_TURNRIGHT:
                return 2;
            case MOVEMENT_SHIFTLEFT:
            case MOVEMENT_SHIFTRIGHT:
                return 3;
            default:
                return 0;
            }
        }

        inline bool isPairInternalSwitch(MovementMode from, MovementMode to) {
            const int g1 = switchGroupOf(from);
            const int g2 = switchGroupOf(to);
            return g1 != 0 && g1 == g2 && from != to;
        }

        inline float dist2Point(const hexapod::Point3D& a, const hexapod::Point3D& b) {
            const float dx = a.x_ - b.x_;
            const float dy = a.y_ - b.y_;
            const float dz = a.z_ - b.z_;
            return dx * dx + dy * dy + dz * dz;
        }

        inline float minZ(const QuadLocations& loc) {
            float mz = loc.p[0].z_;
            for (int i = 1; i < 4; ++i) {
                if (loc.p[i].z_ < mz) mz = loc.p[i].z_;
            }
            return mz;
        }

        inline bool isAirLegAt(const QuadLocations& loc, int leg) {
            constexpr float kAirZThresholdMm = 2.0f;
            const float mz = minZ(loc);
            return loc.p[leg].z_ > mz + kAirZThresholdMm;
        }

        inline int roundUpTo(int v, int step) {
            if (step <= 0) return v;
            const int r = v % step;
            return r == 0 ? v : (v + (step - r));
        }

        inline int alignPhaseDurationMs(int phase, float speed) {
            // 对齐过程使用独立的慢速节奏（更稳）。
            // phase: 0=lift(Z-first), 1=moveXY, 2=lower
            const int base =
                (phase == 1) ? 120 : 60; // moveXY 更长
            if (speed <= 0.0f) speed = 1.0f;
            int ms = static_cast<int>(static_cast<float>(base) / speed);
            if (ms < hexapod::config::movementInterval) ms = hexapod::config::movementInterval;
            return roundUpTo(ms, hexapod::config::movementInterval);
        }

        inline float clamp01(float v) {
            if (v < 0.0f) return 0.0f;
            if (v > 1.0f) return 1.0f;
            return v;
        }

        inline float smoothstep(float t) {
            // 0->1, 且两端速度为 0（更“柔和”，尤其适用于落脚/贴地）
            t = clamp01(t);
            return t * t * (3.0f - 2.0f * t);
        }

        inline QuadLocations lerpLocations(const QuadLocations& a, const QuadLocations& b, float t) {
            QuadLocations out = a;
            for (int i = 0; i < 4; ++i) {
                hexapod::Point3D p = a.p[i];
                hexapod::Point3D diff = b.p[i] - a.p[i];
                p += diff * t;
                out.p[i] = p;
            }
            return out;
        }
        
        inline int entryIndexOf(const QuadMovementTable& table) {
            if (!table.entries || table.entriesCount <= 0 || table.length <= 0) {
                return 0;
            }
            int idx = table.entries[0];
            if (idx < 0) idx = 0;
            if (idx >= table.length) idx = idx % table.length;
            return idx;
        }

        inline QuadLocations makeEntryGround(const QuadLocations& loc) {
            QuadLocations out = loc;
            const float z = minZ(loc);
            for (int i = 0; i < 4; ++i) {
                out.p[i].z_ = z;
            }
            return out;
        }

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
          requestedMode_{mode},
          pendingSwitch_{QUAD_SWITCH_NONE},
          gaitMode_{gait},
          position_{},
          index_{0},
          remainTime_{0},
          speed_{config::defaultSpeed} {
    }

    void QuadMovement::setMode(MovementMode newMode) {
        requestedMode_ = newMode;

        // 若处于过渡中（grounding/aligning），新请求到来则直接打断，按新目标重新规划
        grounding_ = false;
        groundRemainTime_ = 0;
        groundTotalTime_ = 0;
        aligning_ = false;
        alignLegCount_ = 0;
        alignLegPos_ = 0;
        alignPhase_ = 0;
        alignLiftZ_ = 0.0f;

        if (requestedMode_ == mode_) {
            pendingSwitch_ = QUAD_SWITCH_NONE;
            return;
        }

        // ---- 姿态动作 / standby 之间切换：不需要抬腿对齐 ----
        // 这些模式理论上不“换落脚点”，因此不走 entry-ground + 逐腿对齐。
        // 直接切换并用较长 remainTime_ 做一次平滑插值即可（避免“抽搐”）。
        const bool fromPostureOrStandby = (mode_ == MOVEMENT_STANDBY) || isPostureMode(mode_);
        const bool toPostureOrStandby = (requestedMode_ == MOVEMENT_STANDBY) || isPostureMode(requestedMode_);
        if (fromPostureOrStandby && toPostureOrStandby) {
            const QuadMovementTable& tgt = tableForMode(requestedMode_);
            if (!tgt.table || !tgt.entries || tgt.length <= 0) {
                LOG_INFO("[QuadMovement] Error: null movement of mode(%d)!", requestedMode_);
                return;
            }

            // 立即切换 table：保持相位 index_（若长度不同则回到 entry）
            const int tgtEntry = entryIndexOf(tgt);
            if (requestedMode_ == MOVEMENT_STANDBY || mode_ == MOVEMENT_STANDBY || tgt.length != currentTable().length) {
                index_ = tgtEntry;
            } else {
                if (index_ < 0) index_ = 0;
                if (index_ >= tgt.length) index_ = index_ % tgt.length;
            }

            mode_ = requestedMode_;
            pendingSwitch_ = QUAD_SWITCH_NONE;

            // 给一个更长的切换时长（独立于 gait step），让姿态动作切换更柔和
            const int actualSwitchDuration = static_cast<int>(config::movementSwitchDuration / speed_);
            const int actualStepDuration = static_cast<int>(tgt.stepDuration / speed_);
            remainTime_ = actualSwitchDuration > actualStepDuration ? actualSwitchDuration : actualStepDuration;
            return;
        }

        // standby -> 其他：直接做“逐腿对齐到目标 entry”（无需等待 entry）
        if (mode_ == MOVEMENT_STANDBY) {
            const QuadMovementTable& tgt = tableForMode(requestedMode_);
            if (!tgt.table || !tgt.entries || tgt.length <= 0) {
                LOG_INFO("[QuadMovement] Error: null movement of mode(%d)!", requestedMode_);
                return;
            }
            alignToMode_ = requestedMode_;
            alignToIndex_ = entryIndexOf(tgt);
            alignTarget_ = tgt.table[alignToIndex_];

            // 构造对齐腿序：目标 entry 若存在悬空腿，则最后动
            constexpr float kSkipDist2 = 1.0f;
            static const int kOrder[4] = {0, 2, 3, 1};
            int groundLegs[4] {0, 0, 0, 0};
            int airLegs[4] {0, 0, 0, 0};
            int groundCount = 0;
            int airCount = 0;

            for (int i = 0; i < 4; ++i) {
                const int leg = kOrder[i];
                if (dist2Point(position_.p[leg], alignTarget_.p[leg]) <= kSkipDist2) {
                    continue;
                }
                if (isAirLegAt(alignTarget_, leg)) {
                    airLegs[airCount++] = leg;
                } else {
                    groundLegs[groundCount++] = leg;
                }
            }

            alignLegCount_ = 0;
            for (int i = 0; i < groundCount; ++i) alignLegs_[alignLegCount_++] = groundLegs[i];
            for (int i = 0; i < airCount; ++i) alignLegs_[alignLegCount_++] = airLegs[i];

            if (alignLegCount_ <= 0) {
                // 已对齐：直接切到目标模式，从 entry 开始迭代
                mode_ = requestedMode_;
                pendingSwitch_ = QUAD_SWITCH_NONE;
                index_ = alignToIndex_;
                position_ = alignTarget_;
                remainTime_ = 0;
                return;
            }

            aligning_ = true;
            pendingSwitch_ = QUAD_SWITCH_NONE;
            alignLegPos_ = 0;
            alignPhase_ = 0;

            const int leg = alignLegs_[alignLegPos_];
            const float z0 = position_.p[leg].z_;
            const float zt = alignTarget_.p[leg].z_;
            if (isAirLegAt(alignTarget_, leg)) {
                // 目标本身是悬空腿：Z-first 直接抬到目标 Z（避免 +18mm 造成“腿抬太高/夹角怪”）
                alignLiftZ_ = zt;
            } else {
                constexpr float kLiftHeightMm = 18.0f;
                const float zBase = (z0 > zt) ? z0 : zt;
                alignLiftZ_ = zBase + kLiftHeightMm;
            }
            remainTime_ = alignPhaseDurationMs(alignPhase_, speed_);
            alignPhaseTotalTime_ = remainTime_;
            alignPhaseStart_ = position_;
            alignPhaseTarget_ = position_;
            // phase=0: lift(Z-first)
            alignPhaseTarget_.p[leg].z_ = alignLiftZ_;
            return;
        }

        // 非 standby：先等当前模式走到它的 entry，再决定如何切换
        pendingSwitch_ = isPairInternalSwitch(mode_, requestedMode_)
            ? QUAD_SWITCH_WAIT_ENTRY_PAIR
            : QUAD_SWITCH_WAIT_ENTRY_ALIGN;
    }

    void QuadMovement::setGaitMode(QuadGaitMode gait) {
        // 仅在稳定 standby 允许切换步态（避免打断停靠/对齐流程）。
        if (mode_ != MOVEMENT_STANDBY ||
            requestedMode_ != MOVEMENT_STANDBY ||
            pendingSwitch_ != QUAD_SWITCH_NONE ||
            grounding_ ||
            aligning_) {
            LOG_INFO("[QuadMovement] Ignore gait switch: not in stable standby.");
            return;
        }

        gaitMode_ = gait;
    }

    const QuadLocations& QuadMovement::next(int elapsedMs) {
        // elapsedMs<=0 时，退化为一个“默认周期”（与六足一致的用法）
        if (elapsedMs <= 0) {
            const QuadMovementTable& t = currentTable();
            elapsedMs = static_cast<int>(t.stepDuration / speed_);
        }

        // ---- grounding：把旧 entry 的悬空腿落地（entry-ground）----
        if (grounding_) {
            if (groundRemainTime_ <= 0) {
                grounding_ = false;
                // 确保落地结束时位置严格对齐到 groundTarget_（避免最后一帧“戳地”跳变）
                position_ = groundTarget_;
                // 进入逐腿对齐
                const QuadMovementTable& tgt = tableForMode(alignToMode_);
                alignTarget_ = tgt.table[alignToIndex_];

                constexpr float kSkipDist2 = 1.0f;
                static const int kOrder[4] = {0, 2, 3, 1};
                int groundLegs[4] {0, 0, 0, 0};
                int airLegs[4] {0, 0, 0, 0};
                int groundCount = 0;
                int airCount = 0;

                for (int i = 0; i < 4; ++i) {
                    const int leg = kOrder[i];
                    if (dist2Point(position_.p[leg], alignTarget_.p[leg]) <= kSkipDist2) {
                        continue;
                    }
                    if (isAirLegAt(alignTarget_, leg)) {
                        airLegs[airCount++] = leg;
                    } else {
                        groundLegs[groundCount++] = leg;
                    }
                }

                alignLegCount_ = 0;
                for (int i = 0; i < groundCount; ++i) alignLegs_[alignLegCount_++] = groundLegs[i];
                for (int i = 0; i < airCount; ++i) alignLegs_[alignLegCount_++] = airLegs[i];

                if (alignLegCount_ <= 0) {
                    // 对齐完成：切到目标模式，从 entry 开始
                    mode_ = alignToMode_;
                    requestedMode_ = mode_;
                    pendingSwitch_ = QUAD_SWITCH_NONE;
                    index_ = alignToIndex_;
                    position_ = alignTarget_;
                    remainTime_ = 0;
                    return position_;
                }

                aligning_ = true;
                alignLegPos_ = 0;
                alignPhase_ = 0;
                const int leg = alignLegs_[alignLegPos_];
                const float z0 = position_.p[leg].z_;
                const float zt = alignTarget_.p[leg].z_;
                if (isAirLegAt(alignTarget_, leg)) {
                    alignLiftZ_ = zt;
                } else {
                    constexpr float kLiftHeightMm = 18.0f;
                    const float zBase = (z0 > zt) ? z0 : zt;
                    alignLiftZ_ = zBase + kLiftHeightMm;
                }
                remainTime_ = alignPhaseDurationMs(alignPhase_, speed_);
                alignPhaseTotalTime_ = remainTime_;
                alignPhaseStart_ = position_;
                alignPhaseTarget_ = position_;
                alignPhaseTarget_.p[leg].z_ = alignLiftZ_;
                // 继续走 aligning_ 分支（不丢 elapsedMs）
            } else {
                if (elapsedMs >= groundRemainTime_)
                    elapsedMs = groundRemainTime_;
                if (groundTotalTime_ <= 0) {
                    groundRemainTime_ = 0;
                    position_ = groundTarget_;
                    return position_;
                }
                groundRemainTime_ -= elapsedMs;
                const float p = 1.0f - static_cast<float>(groundRemainTime_) / static_cast<float>(groundTotalTime_);
                position_ = lerpLocations(groundStart_, groundTarget_, smoothstep(p));
                return position_;
            }
        }

        // ---- aligning：逐腿对齐到目标 entry（目标悬空腿最后动）----
        if (aligning_) {
            // phase/leg 推进
            if (remainTime_ <= 0) {
                const int leg = alignLegs_[alignLegPos_];
                const bool targetAir = isAirLegAt(alignTarget_, leg);

                if (targetAir) {
                    // 目标为悬空腿：只需要 lift -> moveXY 两段
                    if (alignPhase_ == 0) {
                        alignPhase_ = 1;
                    } else {
                        // 完成当前腿
                        alignLegPos_++;
                        alignPhase_ = 0;
                    }
                } else {
                    // 目标为触地腿：lift -> moveXY -> lower 三段
                    if (alignPhase_ == 0) {
                        alignPhase_ = 1;
                    } else if (alignPhase_ == 1) {
                        alignPhase_ = 2;
                    } else {
                        alignLegPos_++;
                        alignPhase_ = 0;
                    }
                }

                if (alignLegPos_ >= alignLegCount_) {
                    // 对齐完成：切换到目标模式并从 entry 开始迭代
                    aligning_ = false;
                    mode_ = alignToMode_;
                    requestedMode_ = mode_;
                    pendingSwitch_ = QUAD_SWITCH_NONE;
                    index_ = alignToIndex_;
                    position_ = alignTarget_;
                    remainTime_ = 0;
                    return position_;
                }

                // 注意：alignLiftZ_ 只能在“开始一条新腿(phase=0)”时计算。
                // 否则在同一条腿的 phase 0->1->2 过程中重复叠加，会导致抬腿高度越来越高，
                // 最终出现“戳地/砸地”。
                if (alignPhase_ == 0) {
                    const int nextLeg = alignLegs_[alignLegPos_];
                    const float z0 = position_.p[nextLeg].z_;
                    const float zt = alignTarget_.p[nextLeg].z_;
                    if (isAirLegAt(alignTarget_, nextLeg)) {
                        alignLiftZ_ = zt;
                    } else {
                        constexpr float kLiftHeightMm = 18.0f;
                        const float zBase = (z0 > zt) ? z0 : zt;
                        alignLiftZ_ = zBase + kLiftHeightMm;
                    }
                }
                remainTime_ = alignPhaseDurationMs(alignPhase_, speed_);

                // 为当前 phase 设置“起点/终点”，用于平滑插值（尤其是落脚避免“戳地”）
                alignPhaseTotalTime_ = remainTime_;
                alignPhaseStart_ = position_;
                alignPhaseTarget_ = position_;
                const int curLeg = alignLegs_[alignLegPos_];
                if (alignPhase_ == 0) {
                    // lift(Z-first)
                    alignPhaseTarget_.p[curLeg].z_ = alignLiftZ_;
                } else if (alignPhase_ == 1) {
                    // moveXY
                    alignPhaseTarget_.p[curLeg].x_ = alignTarget_.p[curLeg].x_;
                    alignPhaseTarget_.p[curLeg].y_ = alignTarget_.p[curLeg].y_;
                    alignPhaseTarget_.p[curLeg].z_ = alignLiftZ_;
                } else {
                    // lower(to target)
                    alignPhaseTarget_.p[curLeg] = alignTarget_.p[curLeg];
                }
            }

            if (elapsedMs >= remainTime_)
                elapsedMs = remainTime_;
            if (alignPhaseTotalTime_ <= 0) {
                // 兜底：理论上不会发生
                alignPhaseTotalTime_ = remainTime_;
                alignPhaseStart_ = position_;
                alignPhaseTarget_ = position_;
            }

            remainTime_ -= elapsedMs;
            const float p = 1.0f - static_cast<float>(remainTime_) / static_cast<float>(alignPhaseTotalTime_);
            position_ = lerpLocations(alignPhaseStart_, alignPhaseTarget_, smoothstep(p));
            return position_;
        }

        // ---- 正常迭代（含 pending switch 触发）----
        const QuadMovementTable* table = &currentTable();
        int actualStepDuration = static_cast<int>(table->stepDuration / speed_);
        if (elapsedMs <= 0)
            elapsedMs = actualStepDuration;

        // ---- pending switch：等待走到当前模式 entry（仅在帧边界触发，避免“半帧跳变”）----
        if (pendingSwitch_ != QUAD_SWITCH_NONE && requestedMode_ != mode_ && remainTime_ <= 0) {
            const int curEntry = entryIndexOf(*table);
            if (index_ == curEntry) {
                if (pendingSwitch_ == QUAD_SWITCH_WAIT_ENTRY_PAIR) {
                    // 组内切换：在 entry 处瞬时跳表到目标模式的 entry
                    const QuadMovementTable& tgt = tableForMode(requestedMode_);
                    if (tgt.table && tgt.length > 0) {
                        const int tgtEntry = entryIndexOf(tgt);
                        mode_ = requestedMode_;
                        pendingSwitch_ = QUAD_SWITCH_NONE;
                        index_ = tgtEntry;
                        position_ = tgt.table[tgtEntry]; // entry 等效，直接对齐以消除累计误差
                        remainTime_ = 0;                 // 立即进入下一帧
                        table = &currentTable();         // 更新 table/stepDuration
                        actualStepDuration = static_cast<int>(table->stepDuration / speed_);
                    } else {
                        pendingSwitch_ = QUAD_SWITCH_NONE;
                    }
                } else {
                    // 跨组切换：entry-ground -> aligning(to target entry)
                    const QuadMovementTable& tgt = tableForMode(requestedMode_);
                    if (!tgt.table || tgt.length <= 0) {
                        pendingSwitch_ = QUAD_SWITCH_NONE;
                    } else {
                        alignToMode_ = requestedMode_;
                        alignToIndex_ = entryIndexOf(tgt);
                        alignTarget_ = tgt.table[alignToIndex_];

                        // 先把当前 entry 的悬空腿落地（entry-ground）
                        groundTarget_ = makeEntryGround(table->table[curEntry]);
                        // entry-ground 的落地过程也要更慢一些，否则会有“砸地/晃”的感觉
                        constexpr int kGroundMsBase = 120;
                        groundTotalTime_ = roundUpTo(
                            static_cast<int>(static_cast<float>(kGroundMsBase) / speed_),
                            hexapod::config::movementInterval);
                        if (groundTotalTime_ < hexapod::config::movementInterval)
                            groundTotalTime_ = hexapod::config::movementInterval;
                        groundRemainTime_ = groundTotalTime_;
                        groundStart_ = position_;
                        grounding_ = true;
                        pendingSwitch_ = QUAD_SWITCH_NONE;
                        // 立即开始 grounding（不再继续走当前 gait），复用 grounding_ 分支的 smoothstep 插值
                        return next(elapsedMs);
                    }
                }
            }
        }

        // ---- 正常 gait 帧推进 ----
        if (remainTime_ <= 0) {
            index_ = (index_ + 1) % table->length;
            remainTime_ = actualStepDuration;
        }
        if (elapsedMs >= remainTime_)
            elapsedMs = remainTime_;

        QuadLocations target = table->table[index_];
        const float ratio = static_cast<float>(elapsedMs) / static_cast<float>(remainTime_);
        for (int i = 0; i < 4; ++i) {
            hexapod::Point3D diff = target.p[i] - position_.p[i];
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

    float QuadMovement::cycleDurationMsForMode(hexapod::MovementMode mode) const {
        float speed = speed_;
        if (speed < config::minSpeed) {
            speed = config::minSpeed;
        }
        const QuadMovementTable& table = tableForMode(mode);
        return static_cast<float>(table.length) * (static_cast<float>(table.stepDuration) / speed);
    }

    const QuadMovementTable& QuadMovement::currentTable() const {
        return selectTable(gaitMode_, mode_);
    }

    const QuadMovementTable& QuadMovement::tableForMode(MovementMode mode) const {
        return selectTable(gaitMode_, mode);
    }

} // namespace quadruped

#endif // ROBOT_MODEL_NODEQUADMINI
