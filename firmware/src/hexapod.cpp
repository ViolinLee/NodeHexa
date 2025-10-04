#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "hexapod.h"
#include "servo.h"
#include "debug.h"

namespace hexapod {

    HexapodClass Hexapod;

    HexapodClass::HexapodClass(): 
        legs_{{0}, {1}, {2}, {3}, {4}, {5}}, 
        movement_{MOVEMENT_STANDBY},
        mode_{MOVEMENT_STANDBY}
    {

    }

    void HexapodClass::init(bool setting, bool isReset) {
        Servo::init();

        calibrationLoad();

        if (isReset)
            forceResetAllLegTippos();

        // default to standby mode
        if (!setting)
            processMovement(MOVEMENT_STANDBY);

        LOG_INFO("Hexapod init done.");
    }

    void HexapodClass::processMovement(MovementMode mode, int elapsed) {
        if (mode_ != mode) {
            mode_ = mode;
            movement_.setMode(mode_);
        }

        auto& location = movement_.next(elapsed);
        for(int i=0;i<6;i++) {
            legs_[i].moveTip(location.get(i));
        }
    }

    void HexapodClass::setMovementSpeed(float speed) {
        // 受限于舵机频率(50hz->20ms)，速度控制只能是离散的(1/n)
        movement_.setSpeed(speed);
        char buffer[100]; 
        snprintf(buffer, sizeof(buffer), "运动速度已设置为: %.2f (范围: %.1f - %.1f)", 
                 speed, config::minSpeed, config::maxSpeed);
        LOG_INFO(buffer);
    }

    void HexapodClass::setMovementSpeedLevel(SpeedLevel level) {
        if (level < SPEED_SLOWEST || level > SPEED_FAST) {
            LOG_INFO("错误: 无效的速度档位");
            return;
        }
        
        float speed = speedLevelMultipliers[level];
        setMovementSpeed(speed);
        
        const char* levelNames[] = {"慢速", "中速", "快速", "最快"};
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "速度档位已设置为: %s (%.2f)", levelNames[level], speed);
        LOG_INFO(buffer);
    }

    float HexapodClass::getMovementSpeed() const {
        return movement_.getSpeed();
    }

    void HexapodClass::calibrationSave() {
        // {"leg1": [0, 0, 0], ..., "leg6: [0, 0, 0]"}

        StaticJsonDocument<512> doc;

        for(int i=0;i<6;i++) {
            char leg[5];
            sprintf(leg, "leg%d", i);
            JsonArray legData = doc.createNestedArray(leg);
            for(int j=0; j<3; j++) {
                int offset;
                legs_[i].get(j)->getParameter(offset);
                
                Serial.println(offset);
                legData.add((short)offset);
            }
        }

        String output;
        serializeJson(doc, output);
        Serial.println(output);

        File file = SPIFFS.open(calibrationFilePath, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open file for writing");
            return;
        }

        if (serializeJson(doc, file) == 0) {
            Serial.println("Failed to write to file");
        }

        file.close();
    }

    void HexapodClass::calibrationGet(int legIndex, int partIndex, int& offset) {
        legs_[legIndex].get(partIndex)->getParameter(offset);
    }

    void HexapodClass::calibrationSet(int legIndex, int partIndex, int offset) {
        char buffer[100]; 
        snprintf(buffer, sizeof(buffer), "腿部关节舵机校准: 腿部索引[%d] 关节索引[%d] 偏移量[%d]", legIndex, partIndex, offset);
        LOG_INFO(buffer);

        legs_[legIndex].get(partIndex)->setParameter(offset, false);
    }

    void HexapodClass::calibrationSet(CalibrationData&  calibrationData) {
        calibrationSet(calibrationData.legIndex, calibrationData.partIndex, calibrationData.offset);
    }

    void HexapodClass::calibrationTest(int legIndex, int partIndex, float angle) {
        legs_[legIndex].get(partIndex)->setAngle(angle);
    }

    void HexapodClass::calibrationTestAllLeg(float angle) {
        for(int i=0; i<6; i++) {
            for(int j=0; j<3; j++) {
                calibrationTest(i, j, angle);
            }
        }
    }

    void HexapodClass::calibrationLoad() {
        File file = SPIFFS.open(calibrationFilePath, FILE_READ);
        if (!file) {
            Serial.println("[Warn] Failed to open file for reading. Skipping calibration parameters loading!!!");
            return;
        }

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            Serial.print("Failed to read file, using default configuration: ");
            Serial.println(error.c_str());
            return;
        }
        LOG_INFO("Read Servo Motors Calibration Data:");
        serializeJson(doc, Serial);
        Serial.println();

        for (int i = 0; i < 6; i++) {
            char leg[5];
            sprintf(leg, "leg%d", i);
            JsonArray legData = doc[leg];
            for (int j = 0; j < 3; j++) {
                int param = legData[j];
                legs_[i].get(j)->setParameter(param, true);
            }
        }

        file.close();
    }

    void HexapodClass::clearOffset() {
        for(int i=0; i<6; i++) {
            for(int j=0; j<3; j++) {
                legs_[i].get(j)->setParameter(0, true);
            }
        }
    }

    void HexapodClass::forceResetAllLegTippos() {
        for(int i=0; i<6; i++) {
            for(int j=0; j<3; j++) {
                legs_[i].forceResetTipPosition();
            }
        }
    }

}