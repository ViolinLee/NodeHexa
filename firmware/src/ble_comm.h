#pragma once

#include "config.h"

#ifdef USE_BLE_COMM

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "gait_parameters.h"

namespace hexapod {

    // BLE通讯回调接口
    class BLECommCallback {
    public:
        virtual ~BLECommCallback() {}
        
        // 模式切换回调
        virtual void onModeChanged(ControlMode mode) = 0;
        
        // 站立模式控制回调
        virtual void onStandControl(const BodyPose& pose) = 0;
        
        // 行走模式控制回调
        virtual void onWalkControl(const Velocity& vel, float pitch, 
                                  const GaitParameters& params) = 0;
        
        // 特技动作回调
        virtual void onTrickAction(TrickAction action) = 0;
        
        // 紧急停止回调
        virtual void onEmergencyStop() = 0;
        
        // 校准相关回调
        virtual void onCalibrationStart() = 0;
        virtual void onCalibrationAdjust(int legIndex, int partIndex, int offset, float testAngle) = 0;
        virtual void onCalibrationGet(int legIndex, int partIndex) = 0;
        virtual void onCalibrationSave() = 0;
        virtual void onCalibrationExit() = 0;
    };

    // BLE通讯管理器
    class BLEComm {
    public:
        BLEComm();
        
        // 初始化BLE
        void init(const char* deviceName);
        
        // 设置回调
        void setCallback(BLECommCallback* callback);
        
        // 发送状态信息
        void sendStatus(const char* status, int battery, const char* mode, 
                       float voltage, float temperature);
        
        // 发送错误信息
        void sendError(int code, const char* message, const char* level);
        
        // 发送运动状态
        void sendMotionStatus(const char* mode, float vx, float vy, 
                            float vyaw, bool isMoving);
        
        // 发送校准状态
        void sendCalibrationStatus(const char* action, const char* message, 
                                  bool calibrationMode = false);
        
        // 发送校准值
        void sendCalibrationValue(int legIndex, int partIndex, int offset);
        
        // 检查连接状态
        bool isConnected() const;
        
        // 处理接收到的数据
        void process();
        
    private:
        friend class BLEServerCallbacks;
        friend class BLECharacteristicCallbacks;
        
        BLECommCallback* callback_;
        BLEServer* pServer_;
        BLECharacteristic* pTxCharacteristic_;
        BLECharacteristic* pRxCharacteristic_;
        bool deviceConnected_;
        bool oldDeviceConnected_;
        
        // 处理接收到的JSON命令
        void handleCommand(const String& jsonString);
        
        // 发送JSON数据
        void sendJson(const String& jsonString);
    };

    // 全局BLE通讯实例
    extern BLEComm BLE;

} // namespace hexapod

#endif // USE_BLE_COMM

