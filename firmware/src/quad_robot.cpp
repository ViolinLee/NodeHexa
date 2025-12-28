#ifdef ROBOT_MODEL_NODEQUADMINI

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "quad_robot.h"
#include "debug.h"
#include "config.h"

namespace quadruped {

    using namespace hexapod;

    QuadRobot::QuadRobot()
        : speed_{config::defaultSpeed},
          legs_{Leg(0), Leg(1), Leg(2), Leg(3)},
          mode_{MOVEMENT_STANDBY},
          movement_{MOVEMENT_STANDBY, QUAD_GAIT_TROT} {
    }

    void QuadRobot::init(bool setting, bool isReset) {
        ServoQuad::init();
        calibrationLoad();

        if (isReset) {
            forceResetAllLegTippos();
        }

        // 与六足一致：上电初始化不直接 setMode（避免 movementSwitchDuration 造成从 0->standby 的过渡插值，
        // 进而扫过不可达点导致 IK 角度超限/大幅运动）。
        // 直接执行一次 standby，使位置一步对齐到 standby（remainTime_=0 时 ratio=1）。
        mode_ = MOVEMENT_STANDBY;
        if (!setting) {
            processMovement(MOVEMENT_STANDBY, 0);
        }

        LOG_INFO("QuadRobot init done.");
    }

    void QuadRobot::processMovement(MovementMode mode, int elapsedMs) {
        if (mode_ != mode) {
            mode_ = mode;
            movement_.setMode(mode_);
        }

        const QuadLocations& loc = movement_.next(elapsedMs);
        for (int i = 0; i < 4; ++i) {
            legs_[i].moveTip(loc.p[i]);
        }
    }

    void QuadRobot::setMovementSpeed(float speed) {
        if (speed < config::minSpeed)
            speed = config::minSpeed;
        else if (speed > config::maxSpeed)
            speed = config::maxSpeed;

        speed_ = speed;
        movement_.setSpeed(speed_);
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "[Quad] 运动速度已设置为: %.2f (范围: %.1f - %.1f)",
                 speed_, config::minSpeed, config::maxSpeed);
        LOG_INFO(buffer);
    }

    void QuadRobot::setMovementSpeedLevel(SpeedLevel level) {
        if (level < SPEED_SLOWEST || level > SPEED_FAST) {
            LOG_INFO("[Quad] 错误: 无效的速度档位");
            return;
        }

        float speed = speedLevelMultipliers[level];
        setMovementSpeed(speed);

        const char* levelNames[] = {"慢速", "中速", "快速", "最快"};
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "[Quad] 速度档位已设置为: %s (%.2f)", levelNames[level], speed);
        LOG_INFO(buffer);
    }

    float QuadRobot::getMovementSpeed() const {
        return movement_.getSpeed();
    }

    void QuadRobot::calibrationSave() {
        // {"leg0": [0, 0, 0], ..., "leg3": [0, 0, 0]}
        StaticJsonDocument<512> doc;

        for (int i = 0; i < 4; i++) {
            char leg[5];
            sprintf(leg, "leg%d", i);
            JsonArray legData = doc.createNestedArray(leg);
            for (int j = 0; j < 3; j++) {
                int offset;
                legs_[i].get(j)->getParameter(offset);
                legData.add((short)offset);
            }
        }

        File file = SPIFFS.open(kCalibrationFilePath, FILE_WRITE);
        if (!file) {
            LOG_INFO("[Quad] Failed to open calibration file for writing");
            return;
        }

        if (serializeJson(doc, file) == 0) {
            LOG_INFO("[Quad] Failed to write calibration to file");
        }

        file.close();
    }

    void QuadRobot::calibrationGet(int legIndex, int partIndex, int& offset) {
        if (legIndex < 0 || legIndex >= 4 || partIndex < 0 || partIndex >= 3) {
            offset = 0;
            return;
        }
        legs_[legIndex].get(partIndex)->getParameter(offset);
    }

    void QuadRobot::calibrationSet(int legIndex, int partIndex, int offset) {
        if (legIndex < 0 || legIndex >= 4 || partIndex < 0 || partIndex >= 3)
            return;

        char buffer[120];
        snprintf(buffer, sizeof(buffer),
                 "[Quad] 腿部关节舵机校准: 腿部索引[%d] 关节索引[%d] 偏移量[%d]",
                 legIndex, partIndex, offset);
        LOG_INFO(buffer);

        legs_[legIndex].get(partIndex)->setParameter(offset, false);
    }

    void QuadRobot::calibrationSet(CalibrationData& calibrationData) {
        calibrationSet(calibrationData.legIndex, calibrationData.partIndex, calibrationData.offset);
    }

    void QuadRobot::calibrationTest(int legIndex, int partIndex, float angle) {
        if (legIndex < 0 || legIndex >= 4 || partIndex < 0 || partIndex >= 3)
            return;
        legs_[legIndex].get(partIndex)->setAngle(angle);
    }

    void QuadRobot::calibrationTestAllLeg(float angle) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                calibrationTest(i, j, angle);
            }
        }
    }

    void QuadRobot::clearOffset() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                legs_[i].get(j)->setParameter(0, true);
            }
        }
    }

    void QuadRobot::forceResetAllLegTippos() {
        for (int i = 0; i < 4; i++) {
            legs_[i].forceResetTipPosition();
        }
    }

    void QuadRobot::setGaitMode(int gaitMode) {
        QuadGaitMode newGait = QUAD_GAIT_TROT;
        switch (gaitMode) {
        case 1:
            newGait = QUAD_GAIT_WALK;
            break;
        case 2:
            newGait = QUAD_GAIT_GALLOP;
            break;
        case 3:
            newGait = QUAD_GAIT_CREEP;
            break;
        case 0:
        default:
            newGait = QUAD_GAIT_TROT;
            break;
        }

        movement_.setGaitMode(newGait);

        const char* gaitNames[] = {"TROT", "WALK", "GALLOP", "CREEP"};
        int idx = static_cast<int>(newGait);
        if (idx < 0 || idx >= 4) idx = 0;
        char buffer[80];
        snprintf(buffer, sizeof(buffer), "[Quad] 切换步态模式为: %s(%d)", gaitNames[idx], gaitMode);
        LOG_INFO(buffer);
    }

    void QuadRobot::calibrationLoad() {
        File file = SPIFFS.open(kCalibrationFilePath, FILE_READ);
        if (!file) {
            LOG_INFO("[Quad] No calibration file, using default servo parameters.");
            return;
        }

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            LOG_INFO("[Quad] Failed to read calibration file, using default configuration.");
            file.close();
            return;
        }

        LOG_INFO("[Quad] Read Servo Motors Calibration Data:");
        serializeJson(doc, Serial);
        Serial.println();

        for (int i = 0; i < 4; i++) {
            char leg[5];
            sprintf(leg, "leg%d", i);
            JsonArray legData = doc[leg];
            if (legData.isNull())
                continue;
            for (int j = 0; j < 3; j++) {
                int param = legData[j];
                legs_[i].get(j)->setParameter(param, true);
            }
        }

        file.close();
    }

} // namespace quadruped

#endif // ROBOT_MODEL_NODEQUADMINI

