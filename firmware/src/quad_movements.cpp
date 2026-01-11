#ifdef ROBOT_MODEL_NODEQUADMINI

#include "base.h"
#include "movement.h"
#include "quad_tables.h"
#include "config.h"

using namespace hexapod;
using namespace hexapod::config;

// 四足默认足端世界坐标（home 点）：
// 与六足 movements.cpp 一样在本编译单元内定义宏，并在此处提供 standbyTable()。
// 这样 quad_robot.cpp 不需要再“写死/重复推导”，也不需要跨 cpp 共享宏。

#define SIN30   0.5
#define COS30   0.866
#define SIN15   0.2588
#define COS15   0.9659

// 站立高度（正值），世界坐标下 Z 取负
#define Q_STANDBY_Z_POS (kLegJoint3ToTip*COS15-kLegJoint2ToJoint3*SIN30)
// 站立时足端在“本地坐标系”下的水平伸展长度
#define Q_REACH (kLegRootToJoint1+kLegJoint1ToJoint2+(kLegJoint2ToJoint3*COS30)+kLegJoint3ToTip*SIN15)
// 四足站立足端外展角：由 config.h 提供（保证与 pathTool 同源）
// 通过调节该角度可改变站立足端在 X/Y 的展开比例，从而影响稳定裕量分布。
#define Q_REACH_X (Q_REACH * kQuadStanceCos)
#define Q_REACH_Y (Q_REACH * kQuadStanceSin)
// 四足足端 home 的 X/Y 偏移量：安装点 + 外展伸展分解
#define Q_OFFSET_X (kQuadLegMountOtherX + Q_REACH_X)
#define Q_OFFSET_Y (kQuadLegMountOtherY + Q_REACH_Y)

// 注意：Q1..Q4 的顺序与 QuadLocations.p[0..3] 对齐（FR, BR, BL, FL）
// 坐标系与六足一致：X 向右为正，Y 向前为正
#define Q1X     (Q_OFFSET_X)     // FR: (+X, +Y)
#define Q1Y     (Q_OFFSET_Y)
#define Q1Z     (-Q_STANDBY_Z_POS)

#define Q2X     (Q_OFFSET_X)     // BR: (+X, -Y)
#define Q2Y     (-Q_OFFSET_Y)
#define Q2Z     (-Q_STANDBY_Z_POS)

#define Q3X     (-Q_OFFSET_X)    // BL: (-X, -Y)
#define Q3Y     (-Q_OFFSET_Y)
#define Q3Z     (-Q_STANDBY_Z_POS)

#define Q4X     (-Q_OFFSET_X)    // FL: (-X, +Y)
#define Q4Y     (Q_OFFSET_Y)
#define Q4Z     (-Q_STANDBY_Z_POS)

namespace quadruped {
    namespace {
        const QuadLocations kStandby {{
            {Q1X, Q1Y, Q1Z}, {Q2X, Q2Y, Q2Z}, {Q3X, Q3Y, Q3Z}, {Q4X, Q4Y, Q4Z}
        }};
        const int zero = 0;
        const QuadMovementTable standby_table {&kStandby, 1, config::movementInterval, &zero, 1};
    }

    const QuadMovementTable& standbyTable() {
        return standby_table;
    }
}

// 引入由 pathTool 生成的四足动作表
#include "generated/movement_table_quad.h"

#endif // ROBOT_MODEL_NODEQUADMINI

