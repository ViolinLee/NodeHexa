#include "ble_comm.h"

#ifdef USE_BLE_COMM

#include <ArduinoJson.h>
#include "debug.h"

namespace hexapod {

    // BLE服务和特征值UUID
    #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
    #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
    #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

    // BLE服务器回调
    class NodeHexaBLEServerCallbacks: public BLEServerCallbacks {
    public:
        NodeHexaBLEServerCallbacks(BLEComm* comm): comm_(comm) {}
        
        void onConnect(BLEServer* pServer) {
            comm_->deviceConnected_ = true;
            LOG_INFO("BLE客户端已连接");
        }
        
        void onDisconnect(BLEServer* pServer) {
            comm_->deviceConnected_ = false;
            LOG_INFO("BLE客户端已断开");
        }
        
    private:
        BLEComm* comm_;
    };

    // BLE特征值回调
    class NodeHexaBLECharacteristicCallbacks: public BLECharacteristicCallbacks {
    public:
        NodeHexaBLECharacteristicCallbacks(BLEComm* comm): comm_(comm) {}
        
        void onWrite(BLECharacteristic *pCharacteristic) {
            std::string value = pCharacteristic->getValue();
            if (value.length() > 0) {
                String jsonString = String(value.c_str());
                comm_->handleCommand(jsonString);
            }
        }
        
    private:
        BLEComm* comm_;
    };

    // 全局实例
    BLEComm BLE;

    BLEComm::BLEComm():
        callback_(nullptr),
        pServer_(nullptr),
        pTxCharacteristic_(nullptr),
        pRxCharacteristic_(nullptr),
        deviceConnected_(false),
        oldDeviceConnected_(false)
    {
    }

    void BLEComm::init(const char* deviceName) {
        // 创建BLE设备
        BLEDevice::init(deviceName);
        
        // 创建BLE服务器
        pServer_ = BLEDevice::createServer();
        pServer_->setCallbacks(new NodeHexaBLEServerCallbacks(this));
        
        // 创建BLE服务
        BLEService *pService = pServer_->createService(SERVICE_UUID);
        
        // 创建TX特征（通知）
        pTxCharacteristic_ = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_TX,
                                BLECharacteristic::PROPERTY_NOTIFY
                             );
        pTxCharacteristic_->addDescriptor(new BLE2902());
        
        // 创建RX特征（写入）
        pRxCharacteristic_ = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_RX,
                                BLECharacteristic::PROPERTY_WRITE
                             );
        pRxCharacteristic_->setCallbacks(new NodeHexaBLECharacteristicCallbacks(this));
        
        // 启动服务
        pService->start();
        
        // 启动广播
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->setMinPreferred(0x06);
        pAdvertising->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
        
        LOG_INFO("BLE已启动，设备名称: %s", deviceName);
    }

    void BLEComm::setCallback(BLECommCallback* callback) {
        callback_ = callback;
    }

    void BLEComm::sendStatus(const char* status, int battery, const char* mode,
                            float voltage, float temperature) {
        if (!deviceConnected_) return;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "status";
        doc["timestamp"] = millis();
        
        JsonObject data = doc.createNestedObject("data");
        data["status"] = status;
        data["battery"] = battery;
        data["mode"] = mode;
        data["voltage"] = voltage;
        data["temperature"] = temperature;
        
        String output;
        serializeJson(doc, output);
        sendJson(output);
    }

    void BLEComm::sendError(int code, const char* message, const char* level) {
        if (!deviceConnected_) return;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "error";
        doc["timestamp"] = millis();
        
        JsonObject data = doc.createNestedObject("data");
        data["code"] = code;
        data["message"] = message;
        data["level"] = level;
        
        String output;
        serializeJson(doc, output);
        sendJson(output);
        
        LOG_INFO("BLE错误发送: [%d] %s", code, message);
    }

    void BLEComm::sendMotionStatus(const char* mode, float vx, float vy,
                                  float vyaw, bool isMoving) {
        if (!deviceConnected_) return;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "motion_status";
        doc["timestamp"] = millis();
        
        JsonObject data = doc.createNestedObject("data");
        data["mode"] = mode;
        data["vx"] = vx;
        data["vy"] = vy;
        data["vyaw"] = vyaw;
        data["is_moving"] = isMoving;
        
        String output;
        serializeJson(doc, output);
        sendJson(output);
    }

    void BLEComm::sendCalibrationStatus(const char* action, const char* message,
                                       bool calibrationMode) {
        if (!deviceConnected_) return;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "calibration_status";
        doc["timestamp"] = millis();
        
        JsonObject data = doc.createNestedObject("data");
        data["action"] = action;
        data["message"] = message;
        if (calibrationMode) {
            data["calibrationMode"] = calibrationMode;
        }
        
        String output;
        serializeJson(doc, output);
        sendJson(output);
    }

    void BLEComm::sendCalibrationValue(int legIndex, int partIndex, int offset) {
        if (!deviceConnected_) return;
        
        StaticJsonDocument<256> doc;
        doc["type"] = "calibration_status";
        doc["timestamp"] = millis();
        
        JsonObject data = doc.createNestedObject("data");
        data["action"] = "get_response";
        data["legIndex"] = legIndex;
        data["partIndex"] = partIndex;
        data["offset"] = offset;
        
        String output;
        serializeJson(doc, output);
        sendJson(output);
    }

    bool BLEComm::isConnected() const {
        return deviceConnected_;
    }

    void BLEComm::process() {
        // 处理连接状态变化
        if (!deviceConnected_ && oldDeviceConnected_) {
            delay(500);
            pServer_->startAdvertising();
            LOG_INFO("BLE重新开始广播");
            oldDeviceConnected_ = deviceConnected_;
        }
        
        if (deviceConnected_ && !oldDeviceConnected_) {
            oldDeviceConnected_ = deviceConnected_;
        }
    }

    void BLEComm::handleCommand(const String& jsonString) {
        if (!callback_) return;
        
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, jsonString);
        
        if (error) {
            LOG_INFO("BLE JSON解析失败: %s", error.c_str());
            sendError(3002, "Invalid JSON format", "warning");
            return;
        }
        
        const char* type = doc["type"];
        if (!type) {
            sendError(3002, "Missing command type", "warning");
            return;
        }
        
        LOG_DEBUG("BLE收到命令: %s", type);
        
        // 处理不同类型的命令
        if (strcmp(type, "stand_mode") == 0) {
            JsonObject data = doc["data"];
            
            BodyPose pose;
            pose.roll = data["roll"] | 0.0f;
            pose.pitch = data["pitch"] | 0.0f;
            pose.yaw = data["yaw"] | 0.0f;
            pose.z = data["height"] | 0.0f;
            pose.validate();
            
            callback_->onModeChanged(MODE_STAND);
            callback_->onStandControl(pose);
            
        } else if (strcmp(type, "walk_mode") == 0) {
            JsonObject data = doc["data"];
            
            Velocity vel;
            vel.vx = data["vx"] | 0.0f;
            vel.vy = data["vy"] | 0.0f;
            vel.vyaw = data["vyaw"] | 0.0f;
            vel.validate();
            
            float pitch = data["pitch"] | 0.0f;
            
            GaitParameters params;
            params.stride = data["stride"] | config::defaultStride;
            params.liftHeight = data["height"] | config::defaultLiftHeight;
            params.validate();
            
            callback_->onModeChanged(MODE_WALK);
            callback_->onWalkControl(vel, pitch, params);
            
        } else if (strcmp(type, "trick") == 0) {
            JsonObject data = doc["data"];
            const char* action = data["action"];
            
            TrickAction trickAction = TRICK_NONE;
            if (strcmp(action, "trick_a") == 0) {
                trickAction = TRICK_A;
            } else if (strcmp(action, "trick_b") == 0) {
                trickAction = TRICK_B;
            } else if (strcmp(action, "trick_c") == 0) {
                trickAction = TRICK_C;
            } else if (strcmp(action, "trick_d") == 0) {
                trickAction = TRICK_D;
            }
            
            callback_->onModeChanged(MODE_TRICK);
            callback_->onTrickAction(trickAction);
            
        } else if (strcmp(type, "emergency_stop") == 0) {
            callback_->onEmergencyStop();
            
        } else if (strcmp(type, "heartbeat") == 0) {
            // 心跳包，回复状态
            // 由主循环定期发送状态
            
        } else if (strcmp(type, "calibration") == 0) {
            JsonObject data = doc["data"];
            const char* action = data["action"];
            
            if (strcmp(action, "start") == 0) {
                callback_->onCalibrationStart();
                
            } else if (strcmp(action, "adjust") == 0) {
                int legIndex = data["legIndex"] | 0;
                int partIndex = data["partIndex"] | 0;
                int offset = data["offset"] | 0;
                float testAngle = data["testAngle"] | 0.0f;
                callback_->onCalibrationAdjust(legIndex, partIndex, offset, testAngle);
                
            } else if (strcmp(action, "get") == 0) {
                int legIndex = data["legIndex"] | 0;
                int partIndex = data["partIndex"] | 0;
                callback_->onCalibrationGet(legIndex, partIndex);
                
            } else if (strcmp(action, "save") == 0) {
                callback_->onCalibrationSave();
                
            } else if (strcmp(action, "exit") == 0) {
                callback_->onCalibrationExit();
            }
            
        } else {
            sendError(3002, "Unknown command type", "warning");
        }
    }

    void BLEComm::sendJson(const String& jsonString) {
        if (!deviceConnected_ || !pTxCharacteristic_) return;
        
        pTxCharacteristic_->setValue(jsonString.c_str());
        pTxCharacteristic_->notify();
        
        LOG_DEBUG("BLE发送: %s", jsonString.c_str());
    }

} // namespace hexapod

#endif // USE_BLE_COMM

