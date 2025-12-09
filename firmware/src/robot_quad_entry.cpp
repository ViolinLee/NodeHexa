#ifdef ROBOT_MODEL_NODEQUADMINI

#include "robot.h"
#include "quad_robot.h"

namespace hexapod {

    // 四足机型：使用 QuadRobot 作为全局 Robot 实例
    static quad::QuadRobot gQuadRobot;
    RobotBase* Robot = &gQuadRobot;

}

#endif // ROBOT_MODEL_NODEQUADMINI

