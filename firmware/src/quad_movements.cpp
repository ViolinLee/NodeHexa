#include "base.h"
#include "movement.h"

using namespace hexapod;

// 四足默认足端世界坐标（需与 pathTool 中 QuadModel 的 home_x/home_y/home_z 一致）
// QuadModel 中定义:
//   standby_z = -80
//   offset_x = 80
//   offset_y = 80
//   home_x = [offset_x, -offset_x, -offset_x, offset_x]
//   home_y = [-offset_y, -offset_y, offset_y, offset_y]
//   home_z = [standby_z, standby_z, standby_z, standby_z]

#define Q1X     80.0f
#define Q1Y     -80.0f
#define Q1Z     -80.0f

#define Q2X     -80.0f
#define Q2Y     -80.0f
#define Q2Z     -80.0f

#define Q3X     -80.0f
#define Q3Y     80.0f
#define Q3Z     -80.0f

#define Q4X     80.0f
#define Q4Y     80.0f
#define Q4Z     -80.0f

// 引入由 pathTool 生成的四足动作表
#include "movement_table_quad.h"

