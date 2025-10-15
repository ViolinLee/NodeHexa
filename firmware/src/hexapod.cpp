#include <ArduinoJson.h>
#include <SPIFFS.h>

#include "hexapod.h"
#include "servo.h"
#include "debug.h"

#ifdef USE_REALTIME_GAIT
#include "standby_position.h"
#endif

namespace hexapod {

    HexapodClass Hexapod;

    HexapodClass::HexapodClass(): 
        legs_{{0}, {1}, {2}, {3}, {4}, {5}}
#ifdef USE_PREDEFINED_GAIT
        , movement_{MOVEMENT_STANDBY}
        , mode_{MOVEMENT_STANDBY}
#endif
#ifdef USE_REALTIME_GAIT
        , controlMode_{MODE_STAND}
        , realtimeGait_{}
        , poseController_{}
        , currentPose_{}
        , walkModePitch_{0.0}
#endif
    {

    }

    void HexapodClass::init(bool setting, bool isReset) {
        Servo::init();

        calibrationLoad();

        if (isReset)
            forceResetAllLegTippos();

#ifdef USE_PREDEFINED_GAIT
        // default to standby mode
        if (!setting)
            processMovement(MOVEMENT_STANDBY);
#endif

#ifdef USE_REALTIME_GAIT
        // 初始化实时步态
        realtimeGait_.reset();
        controlMode_ = MODE_STAND;
        LOG_INFO("实时步态模式已初始化");
#endif

        LOG_INFO("Hexapod init done.");
    }

#ifdef USE_PREDEFINED_GAIT
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
#endif

#ifdef USE_PREDEFINED_GAIT
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
#endif

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

#ifdef USE_REALTIME_GAIT
    // 实时步态API实现

    void HexapodClass::setControlMode(ControlMode mode) {
        if (controlMode_ != mode) {
            controlMode_ = mode;
            LOG_INFO("控制模式切换: %d", mode);
            
            if (mode == MODE_STAND) {
                // 切换到站立模式，重置速度
                Velocity vel;
                realtimeGait_.setVelocity(vel);
            }
        }
    }

    void HexapodClass::setGaitParameters(const GaitParameters& params) {
        realtimeGait_.setGaitParameters(params);
    }

    void HexapodClass::setVelocity(const Velocity& vel) {
        realtimeGait_.setVelocity(vel);
    }

    void HexapodClass::setBodyPose(const BodyPose& pose) {
        currentPose_ = pose;
        poseController_.setBodyPose(pose);
    }

    void HexapodClass::setBodyPitch(float pitch) {
        walkModePitch_ = pitch;
        if (pitch < -15.0) pitch = -15.0;
        if (pitch > 15.0) pitch = 15.0;
    }

    void HexapodClass::executeTrick(TrickAction action) {
        LOG_INFO("执行特技动作: %d", action);
        
        // 特技动作实现
        switch (action) {
        case TRICK_A:
            // 后半身蹲抬前手
            // TODO: 实现具体动作
            break;
        case TRICK_B:
            // 跳舞
            // TODO: 实现具体动作
            break;
        case TRICK_C:
        case TRICK_D:
            // 预留
            break;
        default:
            break;
        }
    }

    void HexapodClass::updateRealtimeGait(int elapsed) {
        Locations targetPositions;
        
        if (controlMode_ == MODE_STAND) {
            // 站立模式：应用姿态控制
            Locations basePositions = standby::getStandbyLocations();
            targetPositions = poseController_.applyPoseTransform(basePositions);
            
        } else if (controlMode_ == MODE_WALK) {
            // 行走模式：生成实时步态
            targetPositions = realtimeGait_.update(elapsed);
            
            // 如果有pitch，应用姿态变换
            if (walkModePitch_ != 0.0) {
                BodyPose tempPose;
                tempPose.pitch = walkModePitch_;
                
                // 保存当前姿态
                BodyPose savedPose = currentPose_;
                
                // 临时设置pitch姿态
                poseController_.setBodyPose(tempPose);
                targetPositions = poseController_.applyPoseTransform(targetPositions);
                
                // 恢复原姿态
                poseController_.setBodyPose(savedPose);
            }
            
        } else if (controlMode_ == MODE_TRICK) {
            // 特技模式：使用特定轨迹
            // TODO: 实现特技轨迹
            targetPositions = standby::getStandbyLocations();
        }
        
        // 更新所有腿的位置
        for (int i = 0; i < 6; i++) {
            legs_[i].moveTip(targetPositions[i]);
        }
    }

    const GaitParameters& HexapodClass::getGaitParameters() const {
        return realtimeGait_.getGaitParameters();
    }

    const Velocity& HexapodClass::getVelocity() const {
        return realtimeGait_.getVelocity();
    }

    const BodyPose& HexapodClass::getBodyPose() const {
        return currentPose_;
    }

#endif // USE_REALTIME_GAIT

}