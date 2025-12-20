#pragma once

#include "base.h"

namespace quadruped {

    struct QuadLocations {
        hexapod::Point3D p[4];
    };

    struct QuadMovementTable {
        const QuadLocations* table;
        int length;
        int stepDuration;
        const int* entries;
        int entriesCount;
    };

    // ---- 四足离线动作表接口声明（由 pathTool 生成并在 quad_movements.cpp 中引入定义） ----
    const QuadMovementTable& quad_trot_forwardTable();
    const QuadMovementTable& quad_trot_backwardTable();
    const QuadMovementTable& quad_trot_shiftleftTable();
    const QuadMovementTable& quad_trot_shiftrightTable();
    const QuadMovementTable& quad_trot_turnleftTable();
    const QuadMovementTable& quad_trot_turnrightTable();

    const QuadMovementTable& quad_walk_forwardTable();
    const QuadMovementTable& quad_walk_backwardTable();
    const QuadMovementTable& quad_walk_shiftleftTable();
    const QuadMovementTable& quad_walk_shiftrightTable();
    const QuadMovementTable& quad_walk_turnleftTable();
    const QuadMovementTable& quad_walk_turnrightTable();

    const QuadMovementTable& quad_gallop_forwardTable();
    const QuadMovementTable& quad_gallop_backwardTable();
    const QuadMovementTable& quad_gallop_shiftleftTable();
    const QuadMovementTable& quad_gallop_shiftrightTable();
    const QuadMovementTable& quad_gallop_turnleftTable();
    const QuadMovementTable& quad_gallop_turnrightTable();

    const QuadMovementTable& quad_creep_forwardTable();
    const QuadMovementTable& quad_creep_backwardTable();
    const QuadMovementTable& quad_creep_shiftleftTable();
    const QuadMovementTable& quad_creep_shiftrightTable();
    const QuadMovementTable& quad_creep_turnleftTable();
    const QuadMovementTable& quad_creep_turnrightTable();

} // namespace quadruped

